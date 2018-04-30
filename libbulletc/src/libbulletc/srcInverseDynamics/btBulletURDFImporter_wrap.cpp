#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "Bullet3Common/b3FileUtils.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "CommonInterfaces/CommonParameterInterface.h"
#include "Utils/b3ResourcePath.h"
#include "Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "Importers/ImportURDFDemo/URDF2Bullet.h"
#include "Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "btBulletURDFImporter_wrap.h"


BulletURDFImporter* BulletURDFImporter_new() {
	return new BulletURDFImporter(NULL, NULL, 1);
}

bool BulletURDFImporter_loadURDF(BulletURDFImporter* obj, const char* fileName, bool forceFixedBase = false) {
	obj->loadURDF(fileName, forceFixedBase);
}

void BulletURDFImporter_delete(BulletURDFImporter* obj) {
	delete obj;
}