#include <iostream>
#include <PxPhysicsAPI.h>

#include "snippetrender/SnippetRender.h"
#include "snippetrender/SnippetCamera.h"

Snippets::Camera *camera;
physx::PxDefaultAllocator allocator;
physx::PxDefaultErrorCallback errorCallback;
physx::PxFoundation* foundation;
//physx::PxPvd* pvd;
physx::PxPvdTransport* transport;
physx::PxPhysics* physics;
physx::PxScene* scene;
physx::PxArray<physx::PxActor*> removedActors;
physx::PxMaterial* rockMaterial;

class CustomEventCallback : public physx::PxSimulationEventCallback {
    virtual void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count) override {};
    virtual void onWake(physx::PxActor** actors, physx::PxU32 count) override {};
    virtual void onSleep(physx::PxActor** actors, physx::PxU32 count) override {};
    virtual void onContact(const physx::PxContactPairHeader& pairHeader, const physx::PxContactPair* pairs, physx::PxU32 nbPairs) override {
        for (int i = 0; i < nbPairs; i++) {
            const physx::PxContactPair& pair = pairs[i];
            physx::PxContactPairPoint contacts[8];
            uint32_t contactsNum = pair.extractContacts(contacts, 8);
            for (int j = 0; j < contactsNum; j++) {
                physx::PxContactPairPoint &contact = contacts[j];
                std::cout << "Projectile hit at position " << contact.position[0] << ", " << contact.position[1] << ", " << contact.position[2] << "\n";
            }
        }
    };
    virtual void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count) override {
        for (physx::PxU32 i = 0; i < count; i++) {
            const physx::PxTriggerPair& pair = pairs[i];
            switch (pair.status) {
            case physx::PxPairFlag::eNOTIFY_TOUCH_FOUND:
                /*std::cout << "TRIGGER FOUND\n";*/
                removedActors.pushBack(pair.otherActor);
                break;
            /*case physx::PxPairFlag::eNOTIFY_TOUCH_PERSISTS:
                std::cout << "TRIGGER PERSISTS\n";
                break;*/
            case physx::PxPairFlag::eNOTIFY_TOUCH_LOST:
                std::cout << "TRIGGER LOST\n";
                break;
            default:
                break;
            }
        }
    };
    virtual void onAdvance(const physx::PxRigidBody* const* bodyBuffer, const physx::PxTransform* poseBuffer, const physx::PxU32 count) override {};
};

CustomEventCallback customEventCallback;

enum Body {
    BOX = 1,
    OBSTACLE,
    TRIGGER
};

struct CustomFilterShaderConstantBlock {
    bool needCollision = true;
};
CustomFilterShaderConstantBlock constantBlock;

physx::PxFilterFlags CustomSimulationFilterShader (
    physx::PxFilterObjectAttributes attributes0,
    physx::PxFilterData filterData0,
    physx::PxFilterObjectAttributes attributes1,
    physx::PxFilterData filterData1,
    physx::PxPairFlags& pairFlags,
    const void* constantBlock,
    physx::PxU32 constantBlockSize
) {
    if (physx::PxFilterObjectIsTrigger(attributes0) || physx::PxFilterObjectIsTrigger(attributes1)) {
        pairFlags = physx::PxPairFlag::eTRIGGER_DEFAULT;
        return physx::PxFilterFlag::eDEFAULT;
    }

    // при выключении флага выключаем взаимодействие между создаваемыми коробками (будут проходить сквозь друг друга)
    if (constantBlockSize == sizeof(CustomFilterShaderConstantBlock)) {
        const CustomFilterShaderConstantBlock* block = (CustomFilterShaderConstantBlock*)constantBlock;
        if (!block->needCollision) {
            if (filterData0.word0 == Body::BOX && filterData1.word0 == Body::BOX) {
                pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT;
                return physx::PxFilterFlag::eKILL;
            }
        }
    }

    if ((filterData0.word0 == Body::BOX && filterData1.word0 == Body::OBSTACLE) || (filterData0.word0 == Body::OBSTACLE && filterData1.word0 == Body::BOX)) {
        pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT | physx::PxPairFlag::eNOTIFY_TOUCH_FOUND | physx::PxPairFlag::eNOTIFY_CONTACT_POINTS;
        return physx::PxFilterFlag::eDEFAULT;
    }

    pairFlags = physx::PxPairFlag::eCONTACT_DEFAULT;
    return physx::PxFilterFlag::eDEFAULT;
}

void initPhysics() {
    foundation = PxCreateFoundation(PX_PHYSICS_VERSION, allocator, errorCallback);

    physx::PxPvd* pvd = physx::PxCreatePvd(*foundation);
    transport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10000);
    if (pvd && transport) {
        bool success = pvd->connect(*transport, physx::PxPvdInstrumentationFlag::eALL);
        if (!success) {
            std::cerr << "PVD was not created\n";
        }
    }
    physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, physx::PxTolerancesScale(), true, pvd);

    physx::PxSceneDesc sceneDesc = physx::PxSceneDesc(physics->getTolerancesScale());
    sceneDesc.gravity = physx::PxVec3(0.0f, -9.81f, 0.0f);
    sceneDesc.cpuDispatcher = physx::PxDefaultCpuDispatcherCreate(2);
    sceneDesc.filterShader = CustomSimulationFilterShader;
    sceneDesc.filterShaderData = &constantBlock;
    sceneDesc.filterShaderDataSize = sizeof(constantBlock);
    sceneDesc.simulationEventCallback = &customEventCallback;
    scene = physics->createScene(sceneDesc);

    if (pvd && pvd->isConnected()) {
        physx::PxPvdSceneClient* pvdClient = scene->getScenePvdClient();
        if (pvdClient) {
            pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
            pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
            pvdClient->setScenePvdFlag(physx::PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
        }
    }

    rockMaterial = physics->createMaterial(0.5f, 0.5f, 0.1f);
    physx::PxMaterial* metalMaterial = physics->createMaterial(0.15f, 0.15f, 0.1f);
    physx::PxMaterial* iceMaterial = physics->createMaterial(0.028f, 0.028f, 0.1f);

    // для теста триггеров
    //const physx::PxVec3 planeNormal = physx::PxVec3(0.3f, 1.0f, 0.0f).getNormalized();
    const physx::PxVec3 planeNormal = physx::PxVec3(0.0f, 1.0f, 0.0f);
    physx::PxPlane plane = physx::PxPlane(planeNormal, 0.0f);
    physx::PxRigidStatic* groundActor = physx::PxCreatePlane(*physics, plane, *rockMaterial);
    scene->addActor(*groundActor);

    physx::PxBoxGeometry boxGeometry = physx::PxBoxGeometry(physx::PxVec3(0.5f, 0.5f, 0.5f));

    physx::PxShape* rockBoxShape = physics->createShape(boxGeometry, *rockMaterial, true);
    physx::PxRigidDynamic* rockBoxActor = physics->createRigidDynamic(physx::PxTransform(physx::PxVec3(0.0f, 2.0f, -2.0f)));
    rockBoxActor->attachShape(*rockBoxShape);
    physx::PxRigidBodyExt::updateMassAndInertia(*rockBoxActor, 10.0f);
    scene->addActor(*rockBoxActor);

    physx::PxShape* metalBoxShape = physics->createShape(boxGeometry, *metalMaterial, true);
    physx::PxRigidDynamic* metalBoxActor = physics->createRigidDynamic(physx::PxTransform(physx::PxVec3(0.0f, 2.0f, 0.0f)));
    metalBoxActor->attachShape(*metalBoxShape);
    physx::PxRigidBodyExt::updateMassAndInertia(*metalBoxActor, 10.0f);
    scene->addActor(*metalBoxActor);

    physx::PxShape* iceBoxShape = physics->createShape(boxGeometry, *iceMaterial, true);
    physx::PxRigidDynamic* iceBoxActor = physics->createRigidDynamic(physx::PxTransform(physx::PxVec3(0.0f, 2.0f, 2.0f)));
    iceBoxActor->attachShape(*iceBoxShape);
    physx::PxRigidBodyExt::updateMassAndInertia(*iceBoxActor, 10.0f);
    scene->addActor(*iceBoxActor);

    //для теста триггеров
    //physx::PxBoxGeometry obstacleGeometry = physx::PxBoxGeometry(physx::PxVec3(0.5, 0.5, 20.0));
    physx::PxBoxGeometry obstacleGeometry = physx::PxBoxGeometry(physx::PxVec3(2.5, 2.5, 2.5));
    physx::PxShape* obstacleShape = physics->createShape(obstacleGeometry, *rockMaterial, true);
    physx::PxFilterData obstacleFilterData(Body::OBSTACLE, 0, 0, 0);
    obstacleShape->setSimulationFilterData(obstacleFilterData);
    physx::PxRigidStatic* obstacleActor = physics->createRigidStatic(
        physx::PxTransform(physx::PxVec3(0.0, 2.5, 0.0), physx::PxQuat(1.0))
        // для теста триггеров
        //physx::PxTransform(physx::PxVec3(-0.2, 0.0, 0.0), physx::PxQuat(physx::PxPiDivFour, physx::PxVec3(0.0, 0.0, 1.0)))
    );
    obstacleActor->attachShape(*obstacleShape);
    scene->addActor(*obstacleActor);
    
    // триггер, не участвует в физической симуляции
    // для теста триггеров
    //physx::PxBoxGeometry triggerGeometry = physx::PxBoxGeometry(physx::PxVec3(0.5, 20.0, 20.0));
    //physx::PxShape* triggerShape = physics -> createShape(triggerGeometry, *rockMaterial, true, physx::PxShapeFlag::eTRIGGER_SHAPE);
    //physx::PxFilterData triggerFilterData(Body::TRIGGER, 0, 0, 0);
    //triggerShape->setSimulationFilterData(triggerFilterData);
    //physx::PxRigidStatic* triggerActor = physics->createRigidStatic(physx::PxTransform(physx::PxVec3(5.0, 0.0, 0.0)));
    //triggerActor->attachShape(*triggerShape);
    //scene->addActor(*triggerActor);
}

void keyPress(unsigned char key, const physx::PxTransform& cameraTransform)
{
    switch (toupper(key)) {
    case ' ':
        {
            physx::PxSphereGeometry sphereGeometry = physx::PxSphereGeometry(0.25f);
            physx::PxShape* sphereShape = physics->createShape(sphereGeometry, *rockMaterial, true);
            physx::PxFilterData sphereFilterData(Body::BOX, 0, 0, 0);
            sphereShape->setSimulationFilterData(sphereFilterData);
            physx::PxRigidDynamic* sphereActor = physics->createRigidDynamic(physx::PxTransform(cameraTransform.p));
            sphereActor->attachShape(*sphereShape);
            physx::PxRigidBodyExt::updateMassAndInertia(*sphereActor, 10.0f);
            scene->addActor(*sphereActor);
            sphereActor->addForce(camera->getDir() * 10.0f, physx::PxForceMode::eIMPULSE);
            //для теста триггеров
            //physx::PxBoxGeometry boxGeometry = physx::PxBoxGeometry(physx::PxVec3(0.5f, 0.5f, 0.5f));
            //physx::PxShape* boxShape = physics->createShape(boxGeometry, *rockMaterial, true);
            //physx::PxFilterData boxFilterData(Body::BOX, 0, 0, 0);
            //boxShape->setSimulationFilterData(boxFilterData);
            //physx::PxRigidDynamic* boxActor = physics->createRigidDynamic(physx::PxTransform(physx::PxVec3(0.0f, 2.0f, 0.0f)));
            //boxActor->attachShape(*boxShape);
            //physx::PxRigidBodyExt::updateMassAndInertia(*boxActor, 10.0f);
            //scene->addActor(*boxActor);
        }
        break;
    case 'G':
        {
            constantBlock.needCollision = !constantBlock.needCollision;
            scene->setFilterShaderData(&constantBlock, sizeof(constantBlock));
        }
        break;
    default:
        break;
    }

    //std::cout << toupper(key) << "\n";

    /*switch (toupper(key))
    {
    case 'B':	createStack(PxTransform(PxVec3(0, 0, stackZ -= 10.0f)), 10, 2.0f);						break;
    case ' ':	createDynamic(camera, PxSphereGeometry(3.0f), camera.rotate(PxVec3(0, 0, -1)) * 200);	break;
    }*/
}

void renderCallback()
{
    scene->simulate(1.0f / 60.0f);
    scene->fetchResults(true);

    // если раньше не считали коллизии, а теперь включили, то заново считаем шейдеры
    static bool lastNeedCollision = constantBlock.needCollision;
    if (lastNeedCollision != constantBlock.needCollision) {
        if (!lastNeedCollision) {
            physx::PxU32 actorsNum = scene->getNbActors(physx::PxActorTypeFlag::eRIGID_STATIC | physx::PxActorTypeFlag::eRIGID_DYNAMIC);
            if (actorsNum > 0) {
                physx::PxArray<physx::PxRigidActor*> actorsArray(actorsNum);
                scene->getActors(physx::PxActorTypeFlag::eRIGID_STATIC | physx::PxActorTypeFlag::eRIGID_DYNAMIC, (physx::PxActor**)&actorsArray[0], actorsNum);
                for (int i = 0; i < actorsArray.size(); i++) {
                    scene->resetFiltering(*actorsArray[i]);
                }
            }
        }
        lastNeedCollision = constantBlock.needCollision;
    }

    // удаляем объекты, которые соприкоснулись с триггером
    for (int i = 0; i < removedActors.size(); i++) {
        scene->removeActor(*removedActors[i]);
    }
    removedActors.clear();

    Snippets::startRender(camera);

    physx::PxScene* scene;
    PxGetPhysics().getScenes(&scene, 1);

    physx::PxU32 actorsNum = scene->getNbActors(physx::PxActorTypeFlag::eRIGID_STATIC | physx::PxActorTypeFlag::eRIGID_DYNAMIC);
    if (actorsNum > 0) {
        physx::PxArray<physx::PxRigidActor*> actorsArray(actorsNum);
        scene->getActors(physx::PxActorTypeFlag::eRIGID_STATIC | physx::PxActorTypeFlag::eRIGID_DYNAMIC, (physx::PxActor**)&actorsArray[0], actorsNum);
        Snippets::renderActors(&actorsArray[0], (actorsArray.size()), false);
    }

    Snippets::finishRender();
}

void exitCallback() {
    delete camera;
    
    scene->release();
    physics->release();
    foundation->release();
}

int main()
{
    camera = new Snippets::Camera(physx::PxVec3(0.0f, 20.0f, 20.0f), physx::PxVec3(0.0f, -1.0f, -1.0f));
    Snippets::setupDefault("PhysX test", camera, keyPress, renderCallback, exitCallback);

    initPhysics();
    glutMainLoop();
    
    return 0;
}

