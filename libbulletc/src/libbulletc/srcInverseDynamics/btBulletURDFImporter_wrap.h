#include "main.h"
#include "Importers/ImportURDFDemo/BulletUrdfImporter.h"


#ifdef __cplusplus
extern "C" {
#endif

	EXPORT BulletURDFImporter* BulletURDFImporter_new();
	EXPORT bool BulletURDFImporter_loadURDF(BulletURDFImporter* obj, const char* fileName, bool forceFixedBase = false);
	EXPORT void BulletURDFImporter_delete(BulletURDFImporter* obj);


#ifdef __cplusplus
}
#endif
