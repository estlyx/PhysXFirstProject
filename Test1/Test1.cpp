#include <iostream>
#include <array>
#include <cctype>
#include <cstdio>

#include <PxPhysicsAPI.h>

#include "snippetrender/SnippetRender.h"
#include "snippetrender/SnippetCamera.h"

using namespace physx;

// -------------------- Globals --------------------
Snippets::Camera* camera = nullptr;

PxDefaultAllocator allocator;
PxDefaultErrorCallback errorCallback;

PxFoundation* foundation = nullptr;
PxPvd* pvd = nullptr;
PxPvdTransport* transport = nullptr;

PxPhysics* physics = nullptr;
PxScene* scene = nullptr;
PxDefaultCpuDispatcher* dispatcher = nullptr;

PxMaterial* tableMaterial = nullptr;
PxMaterial* railMaterial = nullptr;
PxMaterial* ballMaterial = nullptr;

// Очередь на удаление actor'ов (после fetchResults)
PxArray<PxActor*> removedActors;

// -------------------- Размеры  --------------------
constexpr float kBallR = 0.25f;

constexpr float kTableHalfX = 8.0f;
constexpr float kTableHalfZ = 14.0f;

constexpr float kRailThick = 0.8f;
constexpr float kRailH = 1.0f;

// ---- Лунки (trigger) ----
// Лунки “в бортах”: центр на линии борта (по центру толщины борта),
// а радиус такой, чтобы область триггера заходила внутрь поля.
constexpr float kPocketRadius = (kBallR * 1.6f + kRailThick * 0.6f);

// координаты центров лунок по X/Z на линии бортов
constexpr float kPocketX = (kTableHalfX + kRailThick * 0.5f);
constexpr float kPocketZ = (kTableHalfZ + kRailThick * 0.5f);

// -------------------- Balls --------------------
struct Ball
{
    PxRigidDynamic* actor = nullptr;
    bool isCue = false;
    bool pocketed = false;
    int id = -1;
};

std::array<Ball, 16> balls;
Ball* cueBall = nullptr;

// -------------------- Game state --------------------
bool gameOver = false;
bool printedGameOver = false;
bool winGame = false;

// -------------------- Aim/shot --------------------
float aimAngle = 0.0f;        // 0 => +Z (к пирамиде)
float shotImpulse = 30.0f;
float aimLineLen = 6.0f;

// -------------------- Camera сверху --------------------
float camHeight = 45.0f;
float camBackZ = 0.01f;
PxVec3 camTarget(0.0f, 0.0f, 0.0f);

// -------------------- Filter shader: триггеры + контакты --------------------
PxFilterFlags CustomSimulationFilterShader(
    PxFilterObjectAttributes attributes0, PxFilterData,
    PxFilterObjectAttributes attributes1, PxFilterData,
    PxPairFlags& pairFlags,
    const void*, PxU32)
{
    if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
    {
        pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
        return PxFilterFlag::eDEFAULT;
    }

    pairFlags = PxPairFlag::eCONTACT_DEFAULT;
    return PxFilterFlag::eDEFAULT;
}

// -------------------- Trigger callback --------------------
class CustomEventCallback : public PxSimulationEventCallback
{
public:
    void onTrigger(PxTriggerPair* pairs, PxU32 count) override
    {
        for (PxU32 i = 0; i < count; i++)
        {
            const PxTriggerPair& pair = pairs[i];

            if (!(pair.status & PxPairFlag::eNOTIFY_TOUCH_FOUND))
                continue;

            PxActor* other = pair.otherActor; // шар
            if (!other) continue;

            Ball* b = reinterpret_cast<Ball*>(other->userData);
            if (!b) continue;

            // не добавляем повторно
            if (b->pocketed) continue;

            // нашли столкновение с лункой -> убираем со сцены через removedActors
            b->pocketed = true;
            removedActors.pushBack(other);
        }
    }

    void onConstraintBreak(PxConstraintInfo*, PxU32) override {}
    void onWake(PxActor**, PxU32) override {}
    void onSleep(PxActor**, PxU32) override {}
    void onContact(const PxContactPairHeader&, const PxContactPair*, PxU32) override {}
    void onAdvance(const PxRigidBody* const*, const PxTransform*, const PxU32) override {}
};

CustomEventCallback customEventCallback;

// -------------------- Create helpers --------------------
static PxRigidStatic* createStaticPlane()
{
    PxPlane plane(PxVec3(0, 1, 0), 0.0f);
    PxRigidStatic* groundActor = PxCreatePlane(*physics, plane, *tableMaterial);
    scene->addActor(*groundActor);
    return groundActor;
}

static PxRigidStatic* createStaticBox(const PxVec3& pos, const PxVec3& halfExt, PxMaterial& mat)
{
    PxBoxGeometry geom(halfExt);
    PxShape* shape = physics->createShape(geom, mat, true);

    PxRigidStatic* actor = physics->createRigidStatic(PxTransform(pos));
    actor->attachShape(*shape);

    scene->addActor(*actor);
    return actor;
}

static PxRigidDynamic* createDynamicSphere(const PxVec3& pos, float radius, PxMaterial& mat, float density)
{
    PxSphereGeometry geom(radius);
    PxShape* shape = physics->createShape(geom, mat, true);

    PxRigidDynamic* actor = physics->createRigidDynamic(PxTransform(pos));
    actor->attachShape(*shape);

    PxRigidBodyExt::updateMassAndInertia(*actor, density);

    actor->setLinearDamping(0.12f);
    actor->setAngularDamping(0.35f);
    actor->setSleepThreshold(0.02f);

    scene->addActor(*actor);
    return actor;
}

static PxRigidStatic* createPocketTriggerSphere(const PxVec3& pos)
{
    PxSphereGeometry geom(kPocketRadius);
    PxShape* shape = physics->createShape(geom, *railMaterial, true);

    // trigger shape
    shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
    shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);

    // триггеры не рисуем, так лучше смотрятся
    shape->setFlag(PxShapeFlag::eVISUALIZATION, false);

    PxRigidStatic* actor = physics->createRigidStatic(PxTransform(pos));
    actor->attachShape(*shape);
    scene->addActor(*actor);
    return actor;
}

// -------------------- Table + rails --------------------
static void createTableAndRails()
{
    createStaticPlane();

    // Левый/правый борта вдоль Z
    createStaticBox(
        PxVec3(+(kTableHalfX + kRailThick * 0.5f), kRailH * 0.5f, 0.0f),
        PxVec3(kRailThick * 0.5f, kRailH * 0.5f, kTableHalfZ + kRailThick),
        *railMaterial
    );
    createStaticBox(
        PxVec3(-(kTableHalfX + kRailThick * 0.5f), kRailH * 0.5f, 0.0f),
        PxVec3(kRailThick * 0.5f, kRailH * 0.5f, kTableHalfZ + kRailThick),
        *railMaterial
    );

    // Верх/низ вдоль X
    createStaticBox(
        PxVec3(0.0f, kRailH * 0.5f, +(kTableHalfZ + kRailThick * 0.5f)),
        PxVec3(kTableHalfX + kRailThick, kRailH * 0.5f, kRailThick * 0.5f),
        *railMaterial
    );
    createStaticBox(
        PxVec3(0.0f, kRailH * 0.5f, -(kTableHalfZ + kRailThick * 0.5f)),
        PxVec3(kTableHalfX + kRailThick, kRailH * 0.5f, kRailThick * 0.5f),
        *railMaterial
    );
}

static void createPockets()
{
    const float y = kBallR;

    // 4 угла — в районе углов бортов
    createPocketTriggerSphere(PxVec3(+kPocketX, y, +kPocketZ));
    createPocketTriggerSphere(PxVec3(-kPocketX, y, +kPocketZ));
    createPocketTriggerSphere(PxVec3(+kPocketX, y, -kPocketZ));
    createPocketTriggerSphere(PxVec3(-kPocketX, y, -kPocketZ));

    // центры длинных бортов (длинные вдоль Z, значит центры на x=±)
    createPocketTriggerSphere(PxVec3(+kPocketX + 0.25, y, 0.0f));
    createPocketTriggerSphere(PxVec3(-kPocketX - 0.25, y, 0.0f));
}

// -------------------- Расставляем шары --------------------
static void clearBalls()
{
    for (auto& b : balls)
    {
        if (b.actor)
        {
            scene->removeActor(*b.actor);
            b.actor->release();
            b.actor = nullptr;
        }
        b.isCue = false;
        b.pocketed = false;
        b.id = -1;
    }
    cueBall = nullptr;
}

static void rackBalls()
{
    removedActors.clear();
    clearBalls();

    gameOver = false;
    winGame = false;
    printedGameOver = false;

    const float gap = 0.02f;
    const float dx = (2.0f * kBallR + gap);
    const float dz = (PxSqrt(3.0f) * kBallR + gap);

    const float rackZ = +0.55f * kTableHalfZ;
    const float cueZ = -0.55f * kTableHalfZ;

    int k = 0;
    for (int row = 0; row < 5; ++row)
    {
        for (int j = 0; j <= row; ++j)
        {
            const float x = (j - row * 0.5f) * dx;
            const float z = rackZ + row * dz;

            balls[k].id = k;
            balls[k].isCue = false;
            balls[k].pocketed = false;
            balls[k].actor = createDynamicSphere(PxVec3(x, kBallR, z), kBallR, *ballMaterial, 10.0f);
            balls[k].actor->userData = &balls[k];
            ++k;
        }
    }

    // cue ball (id=15)
    balls[15].id = 15;
    balls[15].isCue = true;
    balls[15].pocketed = false;
    balls[15].actor = createDynamicSphere(PxVec3(0.0f, kBallR, cueZ), kBallR, *ballMaterial, 10.0f);
    balls[15].actor->userData = &balls[15];
    cueBall = &balls[15];

    aimAngle = 0.0f;
    shotImpulse = 30.0f;

    std::cout << "Controls: J/L aim, I/K power, Space shoot, R restart\n";
}

// -------------------- Нарпавление удара --------------------
static PxVec3 aimDir()
{
    const float sx = PxSin(aimAngle);
    const float cz = PxCos(aimAngle);
    PxVec3 d(sx, 0.0f, cz);
    if (d.normalize() < 1e-6f) return PxVec3(0, 0, 1);
    return d;
}

// -------------------- Удар --------------------
static void shoot()
{
    if (gameOver) return;
    if (!cueBall || !cueBall->actor) return;

    PxVec3 d = aimDir();
    cueBall->actor->wakeUp();
    cueBall->actor->addForce(d * shotImpulse, PxForceMode::eIMPULSE);
}

static bool allObjectBallsPocketed()
{
    for (const auto& b : balls)
    {
        if (b.isCue) continue;
        if (!b.pocketed) return false;
    }
    return true;
}

// -------------------- process removals --------------------
static void processRemovals()
{
    if (!removedActors.size())
        return;

    for (PxU32 i = 0; i < removedActors.size(); i++)
    {
        PxActor* a = removedActors[i];
        Ball* b = reinterpret_cast<Ball*>(a->userData);
        if (!b) continue;
        if (!b->actor) continue;

        if (b->isCue)
        {
            gameOver = true;
        }

        scene->removeActor(*b->actor);
        b->actor->release();
        b->actor = nullptr;
    }

    removedActors.clear();

    if (!gameOver && allObjectBallsPocketed())
    {
        gameOver = true;
        winGame = true;
        std::cout << "WIN: all object balls pocketed! Press R to restart.\n";
    }

    if (gameOver && !printedGameOver)
    {
        printedGameOver = true;
        if (winGame)
            std::cout << "WIN: all object balls pocketed! Press R to restart.\n";
        else
            std::cout << "LOSE: cue ball pocketed. Press R to restart.\n";
    }
}

// -------------------- initPhysics --------------------
void initPhysics()
{
    foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);

    pvd = PxCreatePvd(*foundation);
    transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10000);
    if (pvd && transport)
    {
        bool success = pvd->connect(*transport, PxPvdInstrumentationFlag::eALL);
        if (!success)
            std::cerr << "PVD was not created/connected\n";
    }

    physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale(), true, pvd);

    PxSceneDesc sceneDesc(physics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
    dispatcher = PxDefaultCpuDispatcherCreate(2);
    sceneDesc.cpuDispatcher = dispatcher;

    sceneDesc.filterShader = CustomSimulationFilterShader;
    sceneDesc.simulationEventCallback = &customEventCallback;

    scene = physics->createScene(sceneDesc);

    // материалы
    tableMaterial = physics->createMaterial(0.5f, 0.5f, 0.05f);
    railMaterial = physics->createMaterial(0.6f, 0.6f, 0.2f);
    ballMaterial = physics->createMaterial(0.15f, 0.15f, 0.05f);

    createTableAndRails();
    createPockets();
    rackBalls();
}

// -------------------- input --------------------
void keyPress(unsigned char key, const PxTransform&)
{
    switch (std::toupper(key))
    {
    case ' ':
        shoot();
        break;

    case 'L':
        aimAngle -= 0.08f;
        break;
    case 'J':
        aimAngle += 0.08f;
        break;

    case 'I':
        shotImpulse += 2.0f;
        if (shotImpulse > 120.0f) shotImpulse = 120.0f;
        break;
    case 'K':
        shotImpulse -= 2.0f;
        if (shotImpulse < 1.0f) shotImpulse = 1.0f;
        break;

    case 'U': // zoom out
        camHeight += 2.0f;
        camBackZ += 1.0f;
        break;
    case 'O': // zoom in
        camHeight -= 2.0f;
        camBackZ -= 1.0f;
        if (camHeight < 10.0f) camHeight = 10.0f;
        if (camBackZ < 5.0f)  camBackZ = 5.0f;
        break;

    case 'R':
        rackBalls();
        break;

    default:
        break;
    }
}

// -------------------- render --------------------
void renderCallback()
{
    // фиксируем камеру
    PxVec3 eye(0.0f, camHeight, -camBackZ);
    PxVec3 dir = (camTarget - eye).getNormalized();
    camera->setPose(eye, dir);

    scene->simulate(1.0f / 60.0f);
    scene->fetchResults(true);

    processRemovals();

    Snippets::startRender(camera);

    // рисуем actors
    PxU32 actorsNum = scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC);
    if (actorsNum > 0)
    {
        PxArray<PxRigidActor*> actorsArray(actorsNum);
        scene->getActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC,
            (PxActor**)&actorsArray[0], actorsNum);

        Snippets::renderActors(&actorsArray[0], actorsArray.size(), false, physx::PxVec3(0.0f, 0.75f, 0.0f), NULL, false, false);
    }

    // линия прицела
    if (!gameOver && cueBall && cueBall->actor)
    {
        PxVec3 p = cueBall->actor->getGlobalPose().p;
        p.y += 0.05f;

        PxVec3 q = p + aimDir() * aimLineLen;
        Snippets::DrawLine(p, q, PxVec3(1.0f, 0.0f, 0.0f));
    }

    // HUD текст
    {
        char buf[160];
        std::snprintf(buf, sizeof(buf),
            "Aim(J/L): %.2f   Power(I/K): %.1f   %s",
            aimAngle, shotImpulse, gameOver ? "GAME OVER (R to restart)" : "");
        Snippets::print(buf);
    }

    Snippets::finishRender();
}

// -------------------- exit --------------------
void exitCallback()
{
    delete camera;
    camera = nullptr;

    if (scene)
    {
        clearBalls();
        scene->release();
        scene = nullptr;
    }

    if (dispatcher) { dispatcher->release(); dispatcher = nullptr; }

    if (tableMaterial) { tableMaterial->release(); tableMaterial = nullptr; }
    if (railMaterial) { railMaterial->release(); railMaterial = nullptr; }
    if (ballMaterial) { ballMaterial->release(); ballMaterial = nullptr; }

    if (physics) { physics->release(); physics = nullptr; }

    if (transport) { transport->release(); transport = nullptr; }
    if (pvd) { pvd->release(); pvd = nullptr; }
    if (foundation) { foundation->release(); foundation = nullptr; }
}

int main()
{
    camera = new Snippets::Camera(PxVec3(0.0f, camHeight, -camBackZ), PxVec3(0.0f, -1.0f, 0.2f));

    Snippets::setupDefault("PhysX Billiards - pockets + destroy",
        camera, keyPress, renderCallback, exitCallback);

    initPhysics();
    glutMainLoop();
    return 0;
}