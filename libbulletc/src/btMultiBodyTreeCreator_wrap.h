#include "main.h"
#include "BulletInverseDynamics/MultiBodyTree.hpp"
//#include "InverseDynamics/btMultiBodyTreeCreator.hpp"

#ifdef __cplusplus
extern "C" {
#endif
	
	EXPORT btInverseDynamics::btMultiBodyTreeCreator* MultiBodyTreeCreator_new();
	EXPORT int MultiBodyTreeCreator_createFromBtMultiBody(btInverseDynamics::btMultiBodyTreeCreator* obj, btMultiBody* multibody);
	EXPORT btInverseDynamics::MultiBodyTree* MultiBodyTreeCreator_CreateMultiBodyTree(btInverseDynamics::btMultiBodyTreeCreator* obj);
	EXPORT int MultiBodyTreeCreator_getNumBodies(btInverseDynamics::btMultiBodyTreeCreator* obj);
	EXPORT void MultiBodyTreeCreator_delete(btInverseDynamics::MultiBodyTreeCreator* obj);
	
#ifdef __cplusplus
}
#endif
