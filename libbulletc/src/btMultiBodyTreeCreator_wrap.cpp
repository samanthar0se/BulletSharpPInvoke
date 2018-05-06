#include <BulletInverseDynamics/IDConfig.hpp>

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "Bullet3Common/b3FileUtils.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "BulletInverseDynamics/IDConfig.hpp"
#include "InverseDynamics/btMultiBodyTreeCreator.hpp"

#include "btMultiBodyTreeCreator_wrap.h"
 

btInverseDynamics::btMultiBodyTreeCreator* MultiBodyTreeCreator_new() {
	return new btInverseDynamics::btMultiBodyTreeCreator();
}

int MultiBodyTreeCreator_createFromBtMultiBody(btInverseDynamics::btMultiBodyTreeCreator* obj, btMultiBody* multibody) {
	return obj->createFromBtMultiBody(multibody);
}

btInverseDynamics::MultiBodyTree* MultiBodyTreeCreator_CreateMultiBodyTree(btInverseDynamics::btMultiBodyTreeCreator* obj) {
	return btInverseDynamics::CreateMultiBodyTree(*obj);
}

int MultiBodyTreeCreator_getNumBodies(btInverseDynamics::btMultiBodyTreeCreator* obj) {
	int numBodies = 0;
	obj->getNumBodies(&numBodies);
	return numBodies;
}

void MultiBodyTreeCreator_delete(btInverseDynamics::MultiBodyTreeCreator* obj) {
	delete obj;
	
}


