namespace BulletSharp
{
	public class UnsafeNativeMethods
	{
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void AJoint_Prepare(void* obj, int dt, int iterations);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void AJoint_Solve(void* obj, int dt, int sor);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void AJoint_Terminate(void* obj, int dt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void AJoint_Type(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* AllHitsRayResultCallback_new(int^ rayFromWorld, int^ rayToWorld);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void AllHitsRayResultCallback_addSingleResult(void* obj, LocalRayResult^ rayResult, bool normalInWorldSpace);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3AlignedAllocator_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedAllocator_address(void* obj, reference ref);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedAllocator_address(void* obj, const_reference ref);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedAllocator_allocate(void* obj, int n, const_pointer^ hint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedAllocator_construct(void* obj, pointer ptr, value_type^ value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedAllocator_deallocate(void* obj, pointer ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedAllocator_destroy(void* obj, pointer ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3AlignedObjectArray_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3AlignedObjectArray_new(b3AlignedObjectArray^ otherArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_allocate(void* obj, int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_allocSize(void* obj, int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_at(void* obj, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_at(void* obj, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_capacity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_clear(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_copy(void* obj, int start, int end, [unexposed type]^ dest);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_copyFromArray(void* obj, b3AlignedObjectArray^ otherArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_deallocate(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_destroy(void* obj, int first, int last);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_expand(void* obj, [unexposed type]^ fillValue);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_expandNonInitializing(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_findBinarySearch(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_findLinearSearch(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_findLinearSearch2(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_init(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_initializeFromBuffer(void* obj, void^ buffer, int size, int capacity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_operator[](void* obj, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_operator[](void* obj, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_operator=(void* obj, b3AlignedObjectArray^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_pop_back(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_push_back(void* obj, [unexposed type]^ _Val);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_remove(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_reserve(void* obj, int _Count);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_resize(void* obj, int newsize, [unexposed type]^ fillData);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_resizeNoInitialize(void* obj, int newsize);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_size(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AlignedObjectArray_swap(void* obj, int index0, int index1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3AngleCompareFunc_new(int^ anchor);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AngleCompareFunc_operator()(void* obj, b3GrahamVector3^ a, b3GrahamVector3^ b);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3AngularLimit_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AngularLimit_fit(void* obj, int^ angle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AngularLimit_getBiasFactor(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AngularLimit_getCorrection(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AngularLimit_getError(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AngularLimit_getHalfRange(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AngularLimit_getHigh(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AngularLimit_getLow(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AngularLimit_getRelaxationFactor(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AngularLimit_getSign(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AngularLimit_getSoftness(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AngularLimit_isLimit(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AngularLimit_set(void* obj, int low, int high, int _softness, int _biasFactor, int _relaxationFactor);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3AngularLimit_test(void* obj, int angle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3BoundSearchCL_new(int context, int device, int queue, int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3BoundSearchCL_execute(void* obj, b3OpenCLArray^ src, int nSrc, b3OpenCLArray^ dst, int nDst, Option option);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3BoundSearchCL_executeHost(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3BroadphaseAabbCallback_process(void* obj, b3BroadphaseProxy^ proxy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3BroadphasePairSortPredicate_operator()(void* obj, int^ a, int^ b);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3BufferInfoCL_new(int buff, bool isReadOnly);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3BulletFile_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3BulletFile_new(char^ fileName);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3BulletFile_new(char^ memoryBuffer, int len);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3BulletFile_addDataBlock(void* obj, char^ dataBlock);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3BulletFile_addStruct(void* obj, char^ structType, void^ data, int len, void^ oldPtr, int code);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3BulletFile_parse(void* obj, int verboseMode);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3BulletFile_parseData(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3BulletFile_write(void* obj, char^ fileName, bool fixupPointers);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3BulletFile_writeDNA(void* obj, int^ fp);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3CommandLineArgs_new(int argc, char^ argv);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CommandLineArgs_addArgs(void* obj, int argc, char^ argv);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CommandLineArgs_CheckCmdLineFlag(void* obj, char^ arg_name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CommandLineArgs_ParsedArgc(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3Config_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3ConstraintSetting_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3ContactSolverInfo_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ConvexHullComputer_compute(void* obj, void^ coords, bool doubleCoords, int stride, int count, int shrink, int shrinkClamp);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ConvexHullComputer_compute(void* obj, float^ coords, int stride, int count, int shrink, int shrinkClamp);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ConvexHullComputer_compute(void* obj, double^ coords, int stride, int count, int shrink, int shrinkClamp);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3ConvexSeparatingDistanceUtil_new(float boundingRadiusA, float boundingRadiusB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ConvexSeparatingDistanceUtil_getConservativeSeparatingDistance(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ConvexSeparatingDistanceUtil_initSeparatingDistance(void* obj, b3Vector3^ separatingVector, float separatingDistance, b3Transform^ transA, b3Transform^ transB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ConvexSeparatingDistanceUtil_updateSeparatingDistance(void* obj, b3Transform^ transA, b3Transform^ transB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3CpuNarrowPhase_new(b3Config^ config);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_allocateCollidable(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_computeContacts(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_getBodiesCpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_getCollidableCpu(void* obj, int collidableIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_getCollidableCpu(void* obj, int collidableIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_getCollidablesCpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_getInternalData(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_getLocalSpaceAabb(void* obj, int collidableIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_getNumBodiesGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_getNumBodyInertiasGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_getNumCollidablesGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_getNumRigidBodies(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_getObjectTransformFromCpu(void* obj, float^ position, float^ orientation, int bodyIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_getStatic0Index(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_readbackAllBodiesToCpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_registerCompoundShape(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_registerConcaveMesh(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_registerConcaveMeshShape(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_registerConvexHullShape(void* obj, b3ConvexUtility^ utilPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_registerConvexHullShape(void* obj, float^ vertices, int strideInBytes, int numVertices, float^ scaling);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_registerConvexHullShapeInternal(void* obj, b3ConvexUtility^ convexPtr, int^ col);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_registerFace(void* obj, int^ faceNormal, float faceConstant);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_registerPlaneShape(void* obj, int^ planeNormal, float planeConstant);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_registerSphereShape(void* obj, float radius);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_setObjectTransform(void* obj, float^ position, float^ orientation, int bodyIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_setObjectTransformCpu(void* obj, float^ position, float^ orientation, int bodyIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_setObjectVelocityCpu(void* obj, float^ linVel, float^ angVel, int bodyIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuNarrowPhase_writeAllBodiesToGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3CpuRigidBodyPipeline_new(b3CpuNarrowPhase^ narrowphase, b3DynamicBvhBroadphase^ broadphaseDbvt, b3Config^ config);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_addConstraint(void* obj, b3TypedConstraint^ constraint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_allocateCollidable(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_castRays(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_computeContactPoints(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_computeOverlappingPairs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_copyConstraintsToHost(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_createFixedConstraint(void* obj, int bodyA, int bodyB, float^ pivotInA, float^ pivotInB, float^ relTargetAB, float breakingThreshold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_createPoint2PointConstraint(void* obj, int bodyA, int bodyB, float^ pivotInA, float^ pivotInB, float breakingThreshold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_getBodyBuffer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_getNumBodies(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_integrate(void* obj, float timeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_registerConvexPolyhedron(void* obj, b3ConvexUtility^ convex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_registerPhysicsInstance(void* obj, float mass, float^ position, float^ orientation, int collisionShapeIndex, int userData);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_removeConstraint(void* obj, b3TypedConstraint^ constraint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_removeConstraintByUid(void* obj, int uid);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_setGravity(void* obj, float^ grav);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_solveContactConstraints(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_stepSimulation(void* obj, float deltaTime);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_updateAabbWorldSpace(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3CpuRigidBodyPipeline_writeAllInstancesToGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DbvtAabbMm_FromCR(void* obj, int^ c, int r);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DbvtAabbMm_FromMM(void* obj, int^ mi, int^ mx);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DbvtAabbMm_FromPoints(void* obj, int^ pts, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DbvtAabbMm_FromPoints(void* obj, int^ ppts, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3DbvtProxy_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3DbvtProxy_new(int^ aabbMin, int^ aabbMax, void^ userPtr, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3DefaultSerializer_new(int totalSize);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_allocate(void* obj, int size, int numElements);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_finalizeChunk(void* obj, b3Chunk^ chunk, char^ structType, int chunkCode, void^ oldPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_findNameForPointer(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_findPointer(void* obj, void^ oldPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_finishSerialization(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_getBufferPointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_getCurrentBufferSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_getReverseType(void* obj, char^ type);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_getSerializationFlags(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_getUniquePointer(void* obj, void^ oldPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_initDNA(void* obj, char^ bdnaOrg, int dnalen);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_internalAlloc(void* obj, int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_registerNameForPointer(void* obj, void^ ptr, char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_serializeName(void* obj, char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_setSerializationFlags(void* obj, int flags);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_startSerialization(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_writeDNA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DefaultSerializer_writeHeader(void* obj, byte^ buffer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3DynamicBvh_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_benchmark(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_clear(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_clone(void* obj, b3DynamicBvh^ dest, IClone^ iclone);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_collideKDOP(void* obj, b3DbvtNode^ root, int^ normals, int^ offsets, int count, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_collideOCL(void* obj, b3DbvtNode^ root, int^ normals, int^ offsets, int^ sortaxis, int count, ICollide^ policy, bool fullsort);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_collideTT(void* obj, b3DbvtNode^ root0, b3DbvtNode^ root1, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_collideTTpersistentStack(void* obj, b3DbvtNode^ root0, b3DbvtNode^ root1, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_collideTU(void* obj, b3DbvtNode^ root, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_collideTV(void* obj, b3DbvtNode^ root, b3DbvtVolume^ volume, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_countLeaves(void* obj, b3DbvtNode^ node);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_empty(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_enumLeaves(void* obj, b3DbvtNode^ root, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_enumNodes(void* obj, b3DbvtNode^ root, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_extractLeaves(void* obj, b3DbvtNode^ node, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_insert(void* obj, b3DbvtVolume^ box, void^ data);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_maxdepth(void* obj, b3DbvtNode^ node);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_optimizeBottomUp(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_optimizeIncremental(void* obj, int passes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_optimizeTopDown(void* obj, int bu_treshold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_rayTest(void* obj, b3DbvtNode^ root, int^ rayFrom, int^ rayTo, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_rayTestInternal(void* obj, b3DbvtNode^ root, int^ rayFrom, int^ rayTo, int^ rayDirectionInverse, uint^ signs, int lambda_max, int^ aabbMin, int^ aabbMax, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_remove(void* obj, b3DbvtNode^ leaf);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_update(void* obj, b3DbvtNode^ leaf, int lookahead);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_update(void* obj, b3DbvtNode^ leaf, b3DbvtVolume^ volume);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_update(void* obj, b3DbvtNode^ leaf, b3DbvtVolume^ volume, int^ velocity, int margin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_update(void* obj, b3DbvtNode^ leaf, b3DbvtVolume^ volume, int^ velocity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_update(void* obj, b3DbvtNode^ leaf, b3DbvtVolume^ volume, int margin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvh_write(void* obj, IWriter^ iwriter);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3DynamicBvhBroadphase_new(int proxyCapacity, int^ paircache);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_aabbTest(void* obj, int^ aabbMin, int^ aabbMax, b3BroadphaseAabbCallback^ callback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_calculateOverlappingPairs(void* obj, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_collide(void* obj, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_createProxy(void* obj, int^ aabbMin, int^ aabbMax, int objectIndex, void^ userPtr, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_destroyProxy(void* obj, b3BroadphaseProxy^ proxy, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_getAabb(void* obj, int objectId, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_getBroadphaseAabb(void* obj, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_getOverlappingPairCache(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_getOverlappingPairCache(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_getVelocityPrediction(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_optimize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_performDeferredRemoval(void* obj, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_printStats(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_rayTest(void* obj, int^ rayFrom, int^ rayTo, b3BroadphaseRayCallback^ rayCallback, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_resetPool(void* obj, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_setAabb(void* obj, int objectId, int^ aabbMin, int^ aabbMax, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_setAabbForceUpdate(void* obj, b3BroadphaseProxy^ absproxy, int^ aabbMin, int^ aabbMax, int^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3DynamicBvhBroadphase_setVelocityPrediction(void* obj, int prediction);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3FileUtils_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3FileUtils_extractPath(void* obj, char^ fileName, char^ path, int maxPathLength);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3FileUtils_findFile(void* obj, char^ orgFileName, char^ relativeFileName, int maxRelativeFileNameMaxLen);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3FileUtils_strip2(void* obj, char^ name, char^ pattern);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3FileUtils_toLower(void* obj, char^ str);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3FileUtils_toLowerChar(void* obj, char t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3FillCL_new(int ctx, int device, int queue);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3FillCL_execute(void* obj, b3OpenCLArray^ src, uint value, int n, int offset);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3FillCL_execute(void* obj, b3OpenCLArray^ src, int value, int n, int offset);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3FillCL_execute(void* obj, b3OpenCLArray^ src, float value, int n, int offset);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3FillCL_execute(void* obj, int^ src, int^ value, int n, int offset);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3FillCL_executeHost(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3FillCL_executeHost(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GeometryUtil_areVerticesBehindPlane(void* obj, int^ planeNormal, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GeometryUtil_getPlaneEquationsFromVertices(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GeometryUtil_getVerticesFromPlaneEquations(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GeometryUtil_isInside(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GeometryUtil_isPointInsidePlanes(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GjkEpaSolver2_Distance(void* obj, int^ transA, int^ transB, int^ hullA, int^ hullB, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GjkEpaSolver2_Penetration(void* obj, int^ transA, int^ transB, int^ hullA, int^ hullB, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GjkEpaSolver2_StackSizeRequirement(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuBroadphaseInterface_calculateOverlappingPairs(void* obj, int maxPairs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuBroadphaseInterface_calculateOverlappingPairsHost(void* obj, int maxPairs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuBroadphaseInterface_createLargeProxy(void* obj, int^ aabbMin, int^ aabbMax, int userPtr, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuBroadphaseInterface_createProxy(void* obj, int^ aabbMin, int^ aabbMax, int userPtr, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuBroadphaseInterface_getAabbBufferWS(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuBroadphaseInterface_getNumOverlap(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuBroadphaseInterface_getOverlappingPairBuffer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuBroadphaseInterface_writeAabbsToGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3GpuGridBroadphase_new(int ctx, int device, int q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuGridBroadphase_calculateOverlappingPairs(void* obj, int maxPairs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuGridBroadphase_calculateOverlappingPairsHost(void* obj, int maxPairs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuGridBroadphase_CreateFunc(void* obj, int ctx, int device, int q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuGridBroadphase_createLargeProxy(void* obj, int^ aabbMin, int^ aabbMax, int userPtr, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuGridBroadphase_createProxy(void* obj, int^ aabbMin, int^ aabbMax, int userPtr, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuGridBroadphase_getAabbBufferWS(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuGridBroadphase_getNumOverlap(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuGridBroadphase_getOverlappingPairBuffer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuGridBroadphase_writeAabbsToGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3GpuJacobiContactSolver_new(int ctx, int device, int queue, int pairCapacity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuJacobiContactSolver_solveContacts(void* obj, int numBodies, int bodyBuf, int inertiaBuf, int numContacts, int contactBuf, b3Config^ config, int static0Index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuJacobiContactSolver_solveGroupHost(void* obj, int^ bodies, int^ inertias, int numBodies, b3Contact4^ manifoldPtr, int numManifolds, b3JacobiSolverInfo^ solverInfo);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3GpuNarrowPhase_new(int vtx, int dev, int q, b3Config^ config);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_allocateCollidable(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_computeContacts(void* obj, int broadphasePairs, int numBroadphasePairs, int aabbsWorldSpace, int numObjects);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getAabbLocalSpaceBufferGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getBodiesCpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getBodiesGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getBodyInertiasGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getCollidableCpu(void* obj, int collidableIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getCollidableCpu(void* obj, int collidableIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getCollidablesCpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getCollidablesGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getContactsCPU(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getContactsGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getInternalData(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getInternalData(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getLocalSpaceAabb(void* obj, int collidableIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getLocalSpaceAabbsCpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getNumBodiesGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getNumBodyInertiasGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getNumCollidablesGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getNumContactsGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getNumRigidBodies(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getObjectTransformFromCpu(void* obj, float^ position, float^ orientation, int bodyIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_getStatic0Index(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_readbackAllBodiesToCpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_registerCompoundShape(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_registerConcaveMesh(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_registerConcaveMeshShape(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_registerConvexHullShape(void* obj, b3ConvexUtility^ utilPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_registerConvexHullShape(void* obj, float^ vertices, int strideInBytes, int numVertices, float^ scaling);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_registerConvexHullShapeInternal(void* obj, b3ConvexUtility^ convexPtr, int^ col);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_registerFace(void* obj, int^ faceNormal, float faceConstant);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_registerPlaneShape(void* obj, int^ planeNormal, float planeConstant);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_registerRigidBody(void* obj, int collidableIndex, float mass, float^ position, float^ orientation, float^ aabbMin, float^ aabbMax, bool writeToGpu);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_registerSphereShape(void* obj, float radius);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_setObjectTransform(void* obj, float^ position, float^ orientation, int bodyIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_setObjectTransformCpu(void* obj, float^ position, float^ orientation, int bodyIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_setObjectVelocityCpu(void* obj, float^ linVel, float^ angVel, int bodyIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuNarrowPhase_writeAllBodiesToGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3GpuParallelLinearBvh_new(int context, int device, int queue);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuParallelLinearBvh_build(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuParallelLinearBvh_calculateOverlappingPairs(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuParallelLinearBvh_constructBinaryRadixTree(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuParallelLinearBvh_testRaysAgainstBvhAabbs(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3GpuParallelLinearBvhBroadphase_new(int context, int device, int queue);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuParallelLinearBvhBroadphase_calculateOverlappingPairs(void* obj, int maxPairs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuParallelLinearBvhBroadphase_calculateOverlappingPairsHost(void* obj, int maxPairs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuParallelLinearBvhBroadphase_createLargeProxy(void* obj, int^ aabbMin, int^ aabbMax, int userPtr, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuParallelLinearBvhBroadphase_createProxy(void* obj, int^ aabbMin, int^ aabbMax, int userPtr, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuParallelLinearBvhBroadphase_getAabbBufferWS(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuParallelLinearBvhBroadphase_getNumOverlap(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuParallelLinearBvhBroadphase_getOverlappingPairBuffer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuParallelLinearBvhBroadphase_writeAabbsToGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3GpuPgsConstraintSolver_new(int ctx, int device, int queue, bool usePgs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsConstraintSolver_averageVelocities(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsConstraintSolver_initSolverBody(void* obj, int bodyIndex, int^ solverBody, b3RigidBodyData^ rb);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsConstraintSolver_recomputeBatches(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsConstraintSolver_solveGroup(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsConstraintSolver_solveGroupCacheFriendlyFinish(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsConstraintSolver_solveGroupCacheFriendlyIterations(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsConstraintSolver_solveGroupCacheFriendlySetup(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsConstraintSolver_solveJoints(void* obj, int numBodies, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsConstraintSolver_sortConstraintByBatch3(void* obj, b3BatchConstraint^ cs, int numConstraints, int simdWidth, int staticIdx, int numBodies);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3GpuPgsContactSolver_new(int ctx, int device, int q, int pairCapacity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsContactSolver_batchContacts(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsContactSolver_solveContactConstraint(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsContactSolver_solveContactConstraintBatchSizes(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsContactSolver_solveContacts(void* obj, int numBodies, int bodyBuf, int inertiaBuf, int numContacts, int contactBuf, b3Config^ config, int static0Index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsContactSolver_sortConstraintByBatch(void* obj, int^ cs, int n, int simdWidth, int staticIdx, int numBodies);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsContactSolver_sortConstraintByBatch2(void* obj, int^ cs, int n, int simdWidth, int staticIdx, int numBodies);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuPgsContactSolver_sortConstraintByBatch3(void* obj, int^ cs, int n, int simdWidth, int staticIdx, int numBodies, int^ batchSizes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3GpuRaycast_new(int ctx, int device, int q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRaycast_castRays(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRaycast_castRaysHost(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3GpuRigidBodyPipeline_new(int ctx, int device, int q, b3GpuNarrowPhase^ narrowphase, b3GpuBroadphaseInterface^ broadphaseSap, b3DynamicBvhBroadphase^ broadphaseDbvt, int^ config);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_addConstraint(void* obj, b3TypedConstraint^ constraint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_allocateCollidable(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_castRays(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_copyConstraintsToHost(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_createFixedConstraint(void* obj, int bodyA, int bodyB, float^ pivotInA, float^ pivotInB, float^ relTargetAB, float breakingThreshold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_createPoint2PointConstraint(void* obj, int bodyA, int bodyB, float^ pivotInA, float^ pivotInB, float breakingThreshold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_getBodyBuffer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_getNumBodies(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_integrate(void* obj, float timeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_registerConvexPolyhedron(void* obj, b3ConvexUtility^ convex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_registerPhysicsInstance(void* obj, float mass, float^ position, float^ orientation, int collisionShapeIndex, int userData, bool writeInstanceToGpu);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_removeConstraint(void* obj, b3TypedConstraint^ constraint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_removeConstraintByUid(void* obj, int uid);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_setGravity(void* obj, float^ grav);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_setupGpuAabbsFull(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_stepSimulation(void* obj, float deltaTime);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuRigidBodyPipeline_writeAllInstancesToGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3GpuSapBroadphase_new(int ctx, int device, int q, b3GpuSapKernelType kernelType);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_calculateOverlappingPairs(void* obj, int maxPairs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_calculateOverlappingPairsHost(void* obj, int maxPairs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_calculateOverlappingPairsHostIncremental3Sap(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_CreateFuncBarrier(void* obj, int ctx, int device, int q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_CreateFuncBruteForceCpu(void* obj, int ctx, int device, int q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_CreateFuncBruteForceGpu(void* obj, int ctx, int device, int q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_CreateFuncLocalMemory(void* obj, int ctx, int device, int q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_CreateFuncOriginal(void* obj, int ctx, int device, int q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_createLargeProxy(void* obj, b3Vector3^ aabbMin, b3Vector3^ aabbMax, int userPtr, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_createProxy(void* obj, b3Vector3^ aabbMin, b3Vector3^ aabbMax, int userPtr, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_getAabbBufferWS(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_getNumOverlap(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_getOverlappingPairBuffer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_init3dSap(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3GpuSapBroadphase_writeAabbsToGpu(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3GrahamVector3_new(int^ org, int orgIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3HashedOverlappingPairCache_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_cleanOverlappingPair(void* obj, int^ pair, b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_findPair(void* obj, int proxy0, int proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_GetCount(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_getNumOverlappingPairs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_getOverlapFilterCallback(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_getOverlappingPairArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_getOverlappingPairArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_growTables(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_internalAddPair(void* obj, int proxy0, int proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_processAllOverlappingPairs(void* obj, b3OverlapCallback^ , b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_removeOverlappingPair(void* obj, int proxy0, int proxy1, b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_removeOverlappingPairsContainingProxy(void* obj, int proxy, b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashedOverlappingPairCache_setOverlapFilterCallback(void* obj, b3OverlapFilterCallback^ callback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3HashInt_new(int uid);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashInt_equals(void* obj, b3HashInt^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashInt_getHash(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashInt_getUid1(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashInt_setUid1(void* obj, int uid);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3HashKey_new(int uid);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashKey_equals(void* obj, b3HashKey^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashKey_getHash(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashKey_getUid1(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3HashKeyPtr_new(int uid);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashKeyPtr_equals(void* obj, b3HashKeyPtr^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashKeyPtr_getHash(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashKeyPtr_getUid1(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashMap_clear(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashMap_find(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashMap_find(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashMap_findIndex(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashMap_getAtIndex(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashMap_getAtIndex(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashMap_getKeyAtIndex(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashMap_getKeyAtIndex(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashMap_growTables(void* obj, [unexposed type]^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashMap_insert(void* obj, [unexposed type]^ key, [unexposed type]^ value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashMap_operator[](void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashMap_remove(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashMap_size(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3HashPtr_new(void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashPtr_equals(void* obj, b3HashPtr^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashPtr_getHash(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashPtr_getPointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3HashString_new(char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashString_equals(void* obj, b3HashString^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashString_getHash(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3HashString_portableStringCompare(void* obj, char^ src, char^ dst);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3InternalTriangleIndexCallback_internalProcessTriangleIndex(void* obj, int^ triangle, int partId, int triangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3JacobiSolverInfo_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3LauncherCL_new(int queue, int kernel, char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3LauncherCL_deserializeArgs(void* obj, byte^ buf, int bufSize, int ctx);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3LauncherCL_enableSerialization(void* obj, bool serialize);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3LauncherCL_getArgument(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3LauncherCL_getNumArguments(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3LauncherCL_getSerializationBufferSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3LauncherCL_launch1D(void* obj, int numThreads, int localSize);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3LauncherCL_launch2D(void* obj, int numThreadsX, int numThreadsY, int localSizeX, int localSizeY);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3LauncherCL_serializeArguments(void* obj, byte^ destBuffer, int destBufferCapacity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3LauncherCL_serializeToFile(void* obj, char^ fileName, int numWorkItems);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3LauncherCL_setBuffer(void* obj, int clBuffer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3LauncherCL_setBuffers(void* obj, b3BufferInfoCL^ buffInfo, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3LauncherCL_validateResults(void* obj, byte^ goldBuffer, int goldBufferCapacity, int ctx);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3Matrix3x3_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3Matrix3x3_new(b3Quaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3Matrix3x3_new(float^ xx, float^ xy, float^ xz, float^ yx, float^ yy, float^ yz, float^ zx, float^ zy, float^ zz);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3Matrix3x3_new(b3Matrix3x3^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_absolute(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_adjoint(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_cofac(void* obj, int r1, int c1, int r2, int c2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_deSerialize(void* obj, b3Matrix3x3FloatData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_deSerializeDouble(void* obj, b3Matrix3x3DoubleData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_deSerializeFloat(void* obj, b3Matrix3x3FloatData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_determinant(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_diagonalize(void* obj, b3Matrix3x3^ rot, float threshold, int maxSteps);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_getColumn(void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_getEulerYPR(void* obj, float^ yaw, float^ pitch, float^ roll);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_getEulerZYX(void* obj, float^ yaw, float^ pitch, float^ roll, uint solution_number);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_getIdentity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_getOpenGLSubMatrix(void* obj, float^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_getRotation(void* obj, b3Quaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_getRow(void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_inverse(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_operator*=(void* obj, b3Matrix3x3^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_operator[](void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_operator[](void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_operator+=(void* obj, b3Matrix3x3^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_operator=(void* obj, b3Matrix3x3^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_operator-=(void* obj, b3Matrix3x3^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_scaled(void* obj, b3Vector3^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_serialize(void* obj, b3Matrix3x3FloatData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_serializeFloat(void* obj, b3Matrix3x3FloatData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_setEulerYPR(void* obj, float^ yaw, float^ pitch, float^ roll);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_setEulerZYX(void* obj, float eulerX, float eulerY, float eulerZ);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_setFromOpenGLSubMatrix(void* obj, float^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_setIdentity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_setRotation(void* obj, b3Quaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_setValue(void* obj, float^ xx, float^ xy, float^ xz, float^ yx, float^ yy, float^ yz, float^ zx, float^ zy, float^ zz);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_tdotx(void* obj, b3Vector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_tdoty(void* obj, b3Vector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_tdotz(void* obj, b3Vector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_timesTranspose(void* obj, b3Matrix3x3^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_transpose(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Matrix3x3_transposeTimes(void* obj, b3Matrix3x3^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NodeOverlapCallback_processNode(void* obj, int subPart, int triangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NullPairCache_addOverlappingPair(void* obj, int , int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NullPairCache_cleanOverlappingPair(void* obj, int^ , b3Dispatcher^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NullPairCache_cleanProxyFromPairs(void* obj, int , b3Dispatcher^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NullPairCache_findPair(void* obj, int , int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NullPairCache_getNumOverlappingPairs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NullPairCache_getOverlappingPairArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NullPairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NullPairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NullPairCache_hasDeferredRemoval(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NullPairCache_processAllOverlappingPairs(void* obj, b3OverlapCallback^ , b3Dispatcher^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NullPairCache_removeOverlappingPair(void* obj, int , int , b3Dispatcher^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NullPairCache_removeOverlappingPairsContainingProxy(void* obj, int , b3Dispatcher^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NullPairCache_setOverlapFilterCallback(void* obj, b3OverlapFilterCallback^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3NullPairCache_sortOverlappingPairs(void* obj, b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLArray_deallocate(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLArray_operator=(void* obj, b3OpenCLArray^ src);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3OpenCLPlatformInfo_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLUtils_compileCLKernelFromString(void* obj, int clContext, int device, char^ kernelSource, char^ kernelName, int^ pErrNum, int prog, char^ additionalMacros);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLUtils_compileCLProgramFromString(void* obj, int clContext, int device, char^ kernelSource, int^ pErrNum, char^ additionalMacros, char^ srcFileNameForCaching, bool disableBinaryCaching);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLUtils_createContextFromPlatform(void* obj, int platform, int deviceType, int^ pErrNum, void^ pGLCtx, void^ pGLDC, int preferredDeviceIndex, int preferredPlatformIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLUtils_createContextFromType(void* obj, int deviceType, int^ pErrNum, void^ pGLCtx, void^ pGLDC, int preferredDeviceIndex, int preferredPlatformIndex, int^ platformId);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLUtils_getDevice(void* obj, int cxMainContext, int nr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLUtils_getDeviceInfo(void* obj, int device, b3OpenCLDeviceInfo^ info);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLUtils_getNumDevices(void* obj, int cxMainContext);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLUtils_getNumPlatforms(void* obj, int^ pErrNum);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLUtils_getPlatform(void* obj, int nr, int^ pErrNum);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLUtils_getPlatformInfo(void* obj, int platform, b3OpenCLPlatformInfo^ platformInfo);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLUtils_getSdkVendorName(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLUtils_printDeviceInfo(void* obj, int device);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLUtils_printPlatformInfo(void* obj, int platform);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OpenCLUtils_setCachePath(void* obj, char^ path);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlapCallback_processOverlap(void* obj, int^ pair);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlapFilterCallback_needBroadphaseCollision(void* obj, int proxy0, int proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlappingPairCache_addOverlappingPair(void* obj, int proxy0, int proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlappingPairCache_cleanOverlappingPair(void* obj, int^ pair, b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlappingPairCache_cleanProxyFromPairs(void* obj, int proxy, b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlappingPairCache_findPair(void* obj, int proxy0, int proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlappingPairCache_getNumOverlappingPairs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlappingPairCache_getOverlappingPairArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlappingPairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlappingPairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlappingPairCache_hasDeferredRemoval(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlappingPairCache_processAllOverlappingPairs(void* obj, b3OverlapCallback^ , b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlappingPairCache_removeOverlappingPair(void* obj, int proxy0, int proxy1, b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlappingPairCache_removeOverlappingPairsContainingProxy(void* obj, int , b3Dispatcher^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlappingPairCache_setOverlapFilterCallback(void* obj, b3OverlapFilterCallback^ callback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3OverlappingPairCache_sortOverlappingPairs(void* obj, b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ParamsGridBroadphaseCL_getMaxBodiesPerCell(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ParamsGridBroadphaseCL_setMaxBodiesPerCell(void* obj, int maxOverlap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3PgsJacobiSolver_new(bool usePgs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_addFrictionConstraint(void* obj, b3RigidBodyData^ bodies, b3InertiaData^ inertias, int^ normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex, b3ContactPoint^ cp, int^ rel_pos1, int^ rel_pos2, b3RigidBodyData^ colObj0, b3RigidBodyData^ colObj1, int relaxation, int desiredVelocity, int cfmSlip);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_addRollingFrictionConstraint(void* obj, b3RigidBodyData^ bodies, b3InertiaData^ inertias, int^ normalAxis, int solverBodyIdA, int solverBodyIdB, int frictionIndex, b3ContactPoint^ cp, int^ rel_pos1, int^ rel_pos2, b3RigidBodyData^ colObj0, b3RigidBodyData^ colObj1, int relaxation, int desiredVelocity, int cfmSlip);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_averageVelocities(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_B3_DECLARE_ALIGNED_ALLOCATOR(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_b3Rand2(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_b3RandInt2(void* obj, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_convertContact(void* obj, b3RigidBodyData^ bodies, b3InertiaData^ inertias, b3Contact4^ manifold, b3ContactSolverInfo^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_getContactProcessingThreshold(void* obj, b3Contact4^ contact);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_getOrInitSolverBody(void* obj, int bodyIndex, b3RigidBodyData^ bodies, b3InertiaData^ inertias);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_getRandSeed(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_initSolverBody(void* obj, int bodyIndex, int^ solverBody, b3RigidBodyData^ collisionObject);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_resolveSingleConstraintRowGeneric(void* obj, int^ bodyA, int^ bodyB, int^ contactConstraint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_resolveSingleConstraintRowGenericSIMD(void* obj, int^ bodyA, int^ bodyB, int^ contactConstraint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_resolveSingleConstraintRowLowerLimit(void* obj, int^ bodyA, int^ bodyB, int^ contactConstraint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_resolveSingleConstraintRowLowerLimitSIMD(void* obj, int^ bodyA, int^ bodyB, int^ contactConstraint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_resolveSplitPenetrationImpulseCacheFriendly(void* obj, int^ bodyA, int^ bodyB, int^ contactConstraint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_resolveSplitPenetrationSIMD(void* obj, int^ bodyA, int^ bodyB, int^ contactConstraint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_restitutionCurve(void* obj, int rel_vel, int restitution);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_setFrictionConstraintImpulse(void* obj, b3RigidBodyData^ bodies, b3InertiaData^ inertias, int^ solverConstraint, int solverBodyIdA, int solverBodyIdB, b3ContactPoint^ cp, b3ContactSolverInfo^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_setRandSeed(void* obj, ulong seed);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_setupContactConstraint(void* obj, b3RigidBodyData^ bodies, b3InertiaData^ inertias, int^ solverConstraint, int solverBodyIdA, int solverBodyIdB, b3ContactPoint^ cp, b3ContactSolverInfo^ infoGlobal, int^ vel, int^ rel_vel, int^ relaxation, int^ rel_pos1, int^ rel_pos2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_setupFrictionConstraint(void* obj, b3RigidBodyData^ bodies, b3InertiaData^ inertias, int^ solverConstraint, int^ normalAxis, int solverBodyIdA, int solverBodyIdB, b3ContactPoint^ cp, int^ rel_pos1, int^ rel_pos2, b3RigidBodyData^ colObj0, b3RigidBodyData^ colObj1, int relaxation, int desiredVelocity, int cfmSlip);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_setupRollingFrictionConstraint(void* obj, b3RigidBodyData^ bodies, b3InertiaData^ inertias, int^ solverConstraint, int^ normalAxis, int solverBodyIdA, int solverBodyIdB, b3ContactPoint^ cp, int^ rel_pos1, int^ rel_pos2, b3RigidBodyData^ colObj0, b3RigidBodyData^ colObj1, int relaxation, int desiredVelocity, int cfmSlip);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_solveContacts(void* obj, int numBodies, b3RigidBodyData^ bodies, b3InertiaData^ inertias, int numContacts, b3Contact4^ contacts, int numConstraints, int^ constraints);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_solveGroup(void* obj, b3RigidBodyData^ bodies, b3InertiaData^ inertias, int numBodies, b3Contact4^ manifoldPtr, int numManifolds, int^ constraints, int numConstraints, b3ContactSolverInfo^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_solveGroupCacheFriendlyFinish(void* obj, b3RigidBodyData^ bodies, b3InertiaData^ inertias, int numBodies, b3ContactSolverInfo^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_solveGroupCacheFriendlyIterations(void* obj, int^ constraints, int numConstraints, b3ContactSolverInfo^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_solveGroupCacheFriendlySetup(void* obj, b3RigidBodyData^ bodies, b3InertiaData^ inertias, int numBodies, b3Contact4^ manifoldPtr, int numManifolds, int^ constraints, int numConstraints, b3ContactSolverInfo^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_solveGroupCacheFriendlySplitImpulseIterations(void* obj, int^ constraints, int numConstraints, b3ContactSolverInfo^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PgsJacobiSolver_solveSingleIteration(void* obj, int iteration, int^ constraints, int numConstraints, b3ContactSolverInfo^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3PoolAllocator_new(int elemSize, int maxElements);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PoolAllocator_allocate(void* obj, int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PoolAllocator_freeMemory(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PoolAllocator_getElementSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PoolAllocator_getFreeCount(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PoolAllocator_getMaxCount(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PoolAllocator_getPoolAddress(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PoolAllocator_getPoolAddress(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PoolAllocator_getUsedCount(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PoolAllocator_validPtr(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PoolBodyHandle_B3_DECLARE_ALIGNED_ALLOCATOR(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PoolBodyHandle_getNextFree(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PoolBodyHandle_setNextFree(void* obj, int next);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3PrefixScanCL_new(int ctx, int device, int queue, int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PrefixScanCL_execute(void* obj, b3OpenCLArray^ src, b3OpenCLArray^ dst, int n, uint^ sum);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PrefixScanCL_executeHost(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3PrefixScanFloat4CL_new(int ctx, int device, int queue, int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PrefixScanFloat4CL_execute(void* obj, int^ src, int^ dst, int n, int^ sum);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3PrefixScanFloat4CL_executeHost(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3QuadWord_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3QuadWord_new(float^ _x, float^ _y, float^ _z);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3QuadWord_new(float^ _x, float^ _y, float^ _z, float^ _w);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3QuadWord_getX(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3QuadWord_getY(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3QuadWord_getZ(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3QuadWord_operator!=(void* obj, b3QuadWord^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3QuadWord_operator==(void* obj, b3QuadWord^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3QuadWord_setMax(void* obj, b3QuadWord^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3QuadWord_setMin(void* obj, b3QuadWord^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3QuadWord_setValue(void* obj, float^ _x, float^ _y, float^ _z);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3QuadWord_setValue(void* obj, float^ _x, float^ _y, float^ _z, float^ _w);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3QuadWord_setW(void* obj, float _w);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3QuadWord_setX(void* obj, float _x);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3QuadWord_setY(void* obj, float _y);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3QuadWord_setZ(void* obj, float _z);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3Quaternion_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3Quaternion_new(float^ _x, float^ _y, float^ _z, float^ _w);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3Quaternion_new(b3Vector3^ _axis, float^ _angle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3Quaternion_new(float^ yaw, float^ pitch, float^ roll);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_angle(void* obj, b3Quaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_dot(void* obj, b3Quaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_farthest(void* obj, b3Quaternion^ qd);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_getAngle(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_getAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_getEulerZYX(void* obj, float^ yawZ, float^ pitchY, float^ rollX);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_getIdentity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_getW(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_inverse(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_length(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_length2(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_nearest(void* obj, b3Quaternion^ qd);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_normalize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_normalized(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_operator-(void* obj, b3Quaternion^ q2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_operator-(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_operator*(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_operator*=(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_operator*=(void* obj, b3Quaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_operator/(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_operator/=(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_operator+(void* obj, b3Quaternion^ q2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_operator+=(void* obj, b3Quaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_operator-=(void* obj, b3Quaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_setEuler(void* obj, float^ yaw, float^ pitch, float^ roll);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_setEulerZYX(void* obj, float^ yawZ, float^ pitchY, float^ rollX);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_setRotation(void* obj, b3Vector3^ axis, float^ _angle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Quaternion_slerp(void* obj, b3Quaternion^ q, float^ t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3RadixSort32CL_new(int ctx, int device, int queue, int initialCapacity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3RadixSort32CL_execute(void* obj, b3OpenCLArray^ keysIn, b3OpenCLArray^ keysOut, b3OpenCLArray^ valuesIn, b3OpenCLArray^ valuesOut, int n, int sortBits);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3RadixSort32CL_execute(void* obj, b3OpenCLArray^ keysInOut, int sortBits);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3RadixSort32CL_execute(void* obj, b3OpenCLArray^ keyValuesInOut, int sortBits);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3RadixSort32CL_executeHost(void* obj, b3OpenCLArray^ keyValuesInOut, int sortBits);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3RadixSort32CL_executeHost(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3ResizablePool_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ResizablePool_allocHandle(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ResizablePool_exitHandles(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ResizablePool_freeHandle(void* obj, int handle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ResizablePool_getHandle(void* obj, int handle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ResizablePool_getHandle(void* obj, int handle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ResizablePool_getHandleInternal(void* obj, int handle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ResizablePool_getHandleInternal(void* obj, int handle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ResizablePool_getNumHandles(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ResizablePool_getUsedHandles(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ResizablePool_increaseHandleCapacity(void* obj, int extraCapacity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3ResizablePool_initHandles(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3RotationalLimitMotor_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3RotationalLimitMotor_new(b3RotationalLimitMotor^ limot);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3RotationalLimitMotor_isLimited(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3RotationalLimitMotor_needApplyTorques(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3RotationalLimitMotor_solveAngularLimits(void* obj, int timeStep, int^ axis, int jacDiagABInv, b3RigidBodyData^ body0, b3RigidBodyData^ body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3RotationalLimitMotor_testLimitValue(void* obj, int test_value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Serializer_allocate(void* obj, int size, int numElements);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Serializer_finalizeChunk(void* obj, b3Chunk^ chunk, char^ structType, int chunkCode, void^ oldPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Serializer_findNameForPointer(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Serializer_findPointer(void* obj, void^ oldPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Serializer_finishSerialization(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Serializer_getBufferPointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Serializer_getCurrentBufferSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Serializer_getSerializationFlags(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Serializer_getUniquePointer(void* obj, void^ oldPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Serializer_registerNameForPointer(void* obj, void^ ptr, char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Serializer_serializeName(void* obj, char^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Serializer_setSerializationFlags(void* obj, int flags);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Serializer_startSerialization(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3Solver_new(int ctx, int device, int queue, int pairCapacity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Solver_batchContacts(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Solver_convertToConstraints(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Solver_solveContactConstraint(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Solver_solveContactConstraintHost(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3SortedOverlappingPairCache_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_addOverlappingPair(void* obj, int proxy0, int proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_cleanOverlappingPair(void* obj, int^ pair, b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_cleanProxyFromPairs(void* obj, int proxy, b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_findPair(void* obj, int proxy0, int proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_getNumOverlappingPairs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_getOverlapFilterCallback(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_getOverlappingPairArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_getOverlappingPairArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_hasDeferredRemoval(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_needsBroadphaseCollision(void* obj, int proxy0, int proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_processAllOverlappingPairs(void* obj, b3OverlapCallback^ , b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_removeOverlappingPair(void* obj, int proxy0, int proxy1, b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_removeOverlappingPairsContainingProxy(void* obj, int proxy, b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_setOverlapFilterCallback(void* obj, b3OverlapFilterCallback^ callback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SortedOverlappingPairCache_sortOverlappingPairs(void* obj, b3Dispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3StackAlloc_new(uint size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3StackAlloc_allocate(void* obj, uint size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3StackAlloc_beginBlock(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3StackAlloc_create(void* obj, uint size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3StackAlloc_ctor(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3StackAlloc_destroy(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3StackAlloc_endBlock(void* obj, b3Block^ block);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3StackAlloc_getAvailableMemory(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SubSimplexClosestResult_isValid(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SubSimplexClosestResult_reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3SubSimplexClosestResult_setBarycentricCoordinates(void* obj, int a, int b, int c, int d);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3Transform_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3Transform_new(b3Quaternion^ q, b3Vector3^ c);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3Transform_new(b3Matrix3x3^ b, b3Vector3^ c);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3Transform_new(b3Transform^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_deSerialize(void* obj, b3TransformFloatData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_deSerializeDouble(void* obj, b3TransformDoubleData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_deSerializeFloat(void* obj, b3TransformFloatData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_getBasis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_getBasis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_getIdentity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_getOpenGLMatrix(void* obj, float^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_getOrigin(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_getOrigin(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_getRotation(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_inverse(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_inverseTimes(void* obj, b3Transform^ t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_invXform(void* obj, b3Vector3^ inVec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_mult(void* obj, b3Transform^ t1, b3Transform^ t2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_operator()(void* obj, b3Vector3^ x);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_operator*(void* obj, b3Vector3^ x);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_operator*(void* obj, b3Quaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_operator*(void* obj, b3Transform^ t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_operator*=(void* obj, b3Transform^ t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_operator=(void* obj, b3Transform^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_serialize(void* obj, b3TransformFloatData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_serializeFloat(void* obj, b3TransformFloatData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_setBasis(void* obj, b3Matrix3x3^ basis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_setFromOpenGLMatrix(void* obj, float^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_setIdentity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_setOrigin(void* obj, b3Vector3^ origin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Transform_setRotation(void* obj, b3Quaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3TransformUtil_calculateDiffAxisAngle(void* obj, b3Transform^ transform0, b3Transform^ transform1, b3Vector3^ axis, float^ angle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3TransformUtil_calculateDiffAxisAngleQuaternion(void* obj, b3Quaternion^ orn0, b3Quaternion^ orn1a, b3Vector3^ axis, float^ angle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3TransformUtil_calculateVelocity(void* obj, b3Transform^ transform0, b3Transform^ transform1, float timeStep, b3Vector3^ linVel, b3Vector3^ angVel);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3TransformUtil_calculateVelocityQuaternion(void* obj, b3Vector3^ pos0, b3Vector3^ pos1, b3Quaternion^ orn0, b3Quaternion^ orn1, float timeStep, b3Vector3^ linVel, b3Vector3^ angVel);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3TransformUtil_integrateTransform(void* obj, b3Transform^ curTrans, b3Vector3^ linvel, b3Vector3^ angvel, float timeStep, b3Transform^ predictedTransform);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3TranslationalLimitMotor_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3TranslationalLimitMotor_new(b3TranslationalLimitMotor^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3TranslationalLimitMotor_isLimited(void* obj, int limitIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3TranslationalLimitMotor_needApplyForce(void* obj, int limitIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3TranslationalLimitMotor_solveLinearAxis(void* obj, int timeStep, int jacDiagABInv, b3RigidBodyData^ body1, int^ pointInA, b3RigidBodyData^ body2, int^ pointInB, int limit_index, int^ axis_normal_on_a, int^ anchorPos);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3TranslationalLimitMotor_testLimitValue(void* obj, int limitIndex, int test_value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3TriangleCallback_processTriangle(void* obj, int^ triangle, int partId, int triangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3TypedObject_new(int objectType);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3TypedObject_getObjectType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* b3UsageBitfield_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3UsageBitfield_reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_absolute(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_angle(void* obj, b3Vector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_closestAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_cross(void* obj, b3Vector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_deSerialize(void* obj, b3Vector3FloatData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_deSerializeDouble(void* obj, b3Vector3DoubleData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_deSerializeFloat(void* obj, b3Vector3FloatData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_distance(void* obj, b3Vector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_distance2(void* obj, b3Vector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_dot(void* obj, b3Vector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_dot3(void* obj, b3Vector3^ v0, b3Vector3^ v1, b3Vector3^ v2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_furthestAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_fuzzyZero(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_getSkewSymmetricMatrix(void* obj, b3Vector3^ v0, b3Vector3^ v1, b3Vector3^ v2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_getW(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_getX(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_getY(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_getZ(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_isZero(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_length(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_length2(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_lerp(void* obj, b3Vector3^ v, float^ t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_maxAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_maxDot(void* obj, b3Vector3^ array, long array_count, float^ dotOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_minAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_minDot(void* obj, b3Vector3^ array, long array_count, float^ dotOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_normalize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_normalized(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator delete(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator delete(void* obj, void^ , void^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator delete[](void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator delete[](void* obj, void^ , void^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator new(void* obj, int sizeInBytes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator new(void* obj, int size_t, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator new[](void* obj, int sizeInBytes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator new[](void* obj, int size_t, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator!=(void* obj, b3Vector3^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator*=(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator*=(void* obj, b3Vector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator/=(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator+=(void* obj, b3Vector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator-=(void* obj, b3Vector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_operator==(void* obj, b3Vector3^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_rotate(void* obj, b3Vector3^ wAxis, float angle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_safeNormalize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_serialize(void* obj, b3Vector3FloatData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_serializeDouble(void* obj, b3Vector3DoubleData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_serializeFloat(void* obj, b3Vector3FloatData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_setInterpolate3(void* obj, b3Vector3^ v0, b3Vector3^ v1, float rt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_setMax(void* obj, b3Vector3^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_setMin(void* obj, b3Vector3^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_setValue(void* obj, float^ _x, float^ _y, float^ _z);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_setW(void* obj, float _w);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_setX(void* obj, float _x);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_setY(void* obj, float _y);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_setZ(void* obj, float _z);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_setZero(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector3_triple(void* obj, b3Vector3^ v1, b3Vector3^ v2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector4_absolute4(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector4_closestAxis4(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector4_getW(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector4_maxAxis4(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector4_minAxis4(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void b3Vector4_setValue(void* obj, float^ _x, float^ _y, float^ _z, float^ _w);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* bChunkInd_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* bChunkPtr4_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* bChunkPtr8_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* bDNA_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_dumpTypeDefinitions(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_flagEqual(void* obj, int dna_nr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_flagNone(void* obj, int dna_nr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_flagNotEqual(void* obj, int dna_nr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_getArraySize(void* obj, char^ str);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_getArraySizeNew(void* obj, short name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_getElementSize(void* obj, short type, short name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_getLength(void* obj, int ind);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_getName(void* obj, int ind);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_getNumNames(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_getNumStructs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_getPointerSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_getReverseType(void* obj, short type);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_getReverseType(void* obj, char^ type);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_getStruct(void* obj, int ind);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_getType(void* obj, int ind);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_init(void* obj, char^ data, int len, bool swap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_initCmpFlags(void* obj, bDNA^ memDNA);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_initRecurseCmpFlags(void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bDNA_lessThan(void* obj, bDNA^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* bFile_new(char^ filename, char^ headerString);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* bFile_new(char^ memoryBuffer, int len, char^ headerString);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_addDataBlock(void* obj, char^ dataBlock);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_dumpChunks(void* obj, bDNA^ dna);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_findLibPointer(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_getAsString(void* obj, int code);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_getFileDNA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_getFileElement(void* obj, short^ firstStruct, char^ lookupName, char^ lookupType, char^ data, short^ foundPos);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_getFlags(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_getLibPointers(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_getMatchingFileDNA(void* obj, short^ old, char^ lookupName, char^ lookupType, char^ strcData, char^ data, bool fixupPointers);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_getNextBlock(void* obj, bChunkInd^ dataChunk, char^ dataPtr, int flags);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_getVersion(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_ok(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_parse(void* obj, int verboseMode);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_parseData(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_parseHeader(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_parseInternal(void* obj, int verboseMode, char^ memDna, int memDnaLength);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_parseStruct(void* obj, char^ strcPtr, char^ dtPtr, int old_dna, int new_dna, bool fixupPointers);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_preSwap(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_readStruct(void* obj, char^ head, bChunkInd^ chunk);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_resolvePointers(void* obj, int verboseMode);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_resolvePointersChunk(void* obj, bChunkInd^ dataChunk, int verboseMode);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_resolvePointersMismatch(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_resolvePointersStructRecursive(void* obj, char^ strcPtr, int old_dna, int verboseMode, int recursion);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_safeSwapPtr(void* obj, char^ dst, char^ src);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_swap(void* obj, char^ head, bChunkInd^ ch, bool ignoreEndianFlag);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_swapData(void* obj, char^ data, short type, int arraySize, bool ignoreEndianFlag);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_swapDNA(void* obj, char^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_swapLen(void* obj, char^ dataPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_swapStruct(void* obj, int dna_nr, char^ data, bool ignoreEndianFlag);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_updateOldPointers(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_write(void* obj, char^ fileName, bool fixupPointers);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_writeChunks(void* obj, int^ fp, bool fixupPointers);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_writeDNA(void* obj, int^ fp);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void bFile_writeFile(void* obj, char^ fileName);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* Body_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* Body_new(Cluster^ p);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* Body_new(int^ colObj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_activate(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_angularVelocity(void* obj, int^ rpos);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_angularVelocity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_applyAImpulse(void* obj, Impulse^ impulse);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_applyDAImpulse(void* obj, int^ impulse);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_applyDCImpulse(void* obj, int^ impulse);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_applyDImpulse(void* obj, int^ impulse, int^ rpos);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_applyImpulse(void* obj, Impulse^ impulse, int^ rpos);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_applyVAImpulse(void* obj, int^ impulse);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_applyVImpulse(void* obj, int^ impulse, int^ rpos);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_invMass(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_invWorldInertia(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_linearVelocity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_velocity(void* obj, int^ rpos);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Body_xform(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* bt32BitAxisSweep3_new(int^ worldAabbMin, int^ worldAabbMax, uint maxHandles, btOverlappingPairCache^ pairCache, bool disableRaycastAccelerator);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btActionInterface_debugDraw(void* obj, btIDebugDraw^ debugDrawer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btActionInterface_getFixedBody(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btActionInterface_updateAction(void* obj, btCollisionWorld^ collisionWorld, int deltaTimeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btActivatingCollisionAlgorithm_new(int^ ci);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btActivatingCollisionAlgorithm_new(int^ ci, int^ body0Wrap, int^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btAlignedAllocator_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedAllocator_address(void* obj, reference ref);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedAllocator_address(void* obj, const_reference ref);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedAllocator_allocate(void* obj, int n, const_pointer^ hint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedAllocator_construct(void* obj, pointer ptr, value_type^ value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedAllocator_deallocate(void* obj, pointer ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedAllocator_destroy(void* obj, pointer ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btAlignedObjectArray_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btAlignedObjectArray_new(btAlignedObjectArray^ otherArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_allocate(void* obj, int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_allocSize(void* obj, int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_at(void* obj, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_at(void* obj, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_capacity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_clear(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_copy(void* obj, int start, int end, [unexposed type]^ dest);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_copyFromArray(void* obj, btAlignedObjectArray^ otherArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_deallocate(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_destroy(void* obj, int first, int last);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_expand(void* obj, [unexposed type]^ fillValue);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_expandNonInitializing(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_findBinarySearch(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_findLinearSearch(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_findLinearSearch2(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_init(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_initializeFromBuffer(void* obj, void^ buffer, int size, int capacity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_operator[](void* obj, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_operator[](void* obj, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_operator=(void* obj, btAlignedObjectArray^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_pop_back(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_push_back(void* obj, [unexposed type]^ _Val);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_remove(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_removeAtIndex(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_reserve(void* obj, int _Count);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_resize(void* obj, int newsize, [unexposed type]^ fillData);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_resizeNoInitialize(void* obj, int newsize);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_size(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAlignedObjectArray_swap(void* obj, int index0, int index1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btAngleCompareFunc_new(btVector3^ anchor);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAngleCompareFunc_operator()(void* obj, GrahamVector3^ a, GrahamVector3^ b);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btAngularLimit_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAngularLimit_fit(void* obj, int^ angle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAngularLimit_getBiasFactor(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAngularLimit_getCorrection(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAngularLimit_getError(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAngularLimit_getHalfRange(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAngularLimit_getHigh(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAngularLimit_getLow(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAngularLimit_getRelaxationFactor(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAngularLimit_getSign(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAngularLimit_getSoftness(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAngularLimit_isLimit(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAngularLimit_set(void* obj, int low, int high, int _softness, int _biasFactor, int _relaxationFactor);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAngularLimit_test(void* obj, int angle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btAxisSweep3_new(int^ worldAabbMin, int^ worldAabbMax, ushort maxHandles, btOverlappingPairCache^ pairCache, bool disableRaycastAccelerator);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btAxisSweep3Internal_new(int^ worldAabbMin, int^ worldAabbMax, [unexposed type] handleMask, [unexposed type] handleSentinel, [unexposed type] maxHandles, btOverlappingPairCache^ pairCache, bool disableRaycastAccelerator);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_aabbTest(void* obj, int^ aabbMin, int^ aabbMax, btBroadphaseAabbCallback^ callback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_addHandle(void* obj, int^ aabbMin, int^ aabbMax, void^ pOwner, int collisionFilterGroup, int collisionFilterMask, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_allocHandle(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_BT_DECLARE_ALIGNED_ALLOCATOR(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_calculateOverlappingPairs(void* obj, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_createProxy(void* obj, int^ aabbMin, int^ aabbMax, int shapeType, void^ userPtr, int collisionFilterGroup, int collisionFilterMask, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_destroyProxy(void* obj, btBroadphaseProxy^ proxy, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_freeHandle(void* obj, [unexposed type] handle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_getAabb(void* obj, btBroadphaseProxy^ proxy, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_getBroadphaseAabb(void* obj, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_getNumHandles(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_getOverlappingPairCache(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_getOverlappingPairCache(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_getOverlappingPairUserCallback(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_printStats(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_processAllOverlappingPairs(void* obj, btOverlapCallback^ callback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_quantize(void* obj, [unexposed type]^ out, int^ point, int isMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_rayTest(void* obj, int^ rayFrom, int^ rayTo, btBroadphaseRayCallback^ rayCallback, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_removeHandle(void* obj, [unexposed type] handle, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_setAabb(void* obj, btBroadphaseProxy^ proxy, int^ aabbMin, int^ aabbMax, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_setOverlappingPairUserCallback(void* obj, btOverlappingPairCallback^ pairCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_sortMaxDown(void* obj, int axis, [unexposed type] edge, btDispatcher^ dispatcher, bool updateOverlaps);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_sortMaxUp(void* obj, int axis, [unexposed type] edge, btDispatcher^ dispatcher, bool updateOverlaps);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_sortMinDown(void* obj, int axis, [unexposed type] edge, btDispatcher^ dispatcher, bool updateOverlaps);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_sortMinUp(void* obj, int axis, [unexposed type] edge, btDispatcher^ dispatcher, bool updateOverlaps);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_testAabbOverlap(void* obj, btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_testOverlap2D(void* obj, Handle^ pHandleA, Handle^ pHandleB, int axis0, int axis1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_unQuantize(void* obj, btBroadphaseProxy^ proxy, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btAxisSweep3Internal_updateHandle(void* obj, [unexposed type] handle, int^ aabbMin, int^ aabbMax, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btBox2dBox2dCollisionAlgorithm_new(int^ ci);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btBox2dBox2dCollisionAlgorithm_new(btPersistentManifold^ mf, int^ ci, int^ body0Wrap, int^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBox2dBox2dCollisionAlgorithm_calculateTimeOfImpact(void* obj, int^ body0, int^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBox2dBox2dCollisionAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBox2dBox2dCollisionAlgorithm_processCollision(void* obj, int^ body0Wrap, int^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btBoxBoxCollisionAlgorithm_new(int^ ci);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btBoxBoxCollisionAlgorithm_new(btPersistentManifold^ mf, int^ ci, int^ body0Wrap, int^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBoxBoxCollisionAlgorithm_calculateTimeOfImpact(void* obj, int^ body0, int^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBoxBoxCollisionAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBoxBoxCollisionAlgorithm_processCollision(void* obj, int^ body0Wrap, int^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btBoxBoxDetector_new(btBoxShape^ box1, btBoxShape^ box2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBoxBoxDetector_getClosestPoints(void* obj, int^ input, int^ output, btIDebugDraw^ debugDraw, bool swapResults);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBroadphaseAabbCallback_process(void* obj, btBroadphaseProxy^ proxy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBroadphaseInterface_aabbTest(void* obj, int^ aabbMin, int^ aabbMax, btBroadphaseAabbCallback^ callback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBroadphaseInterface_calculateOverlappingPairs(void* obj, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBroadphaseInterface_createProxy(void* obj, int^ aabbMin, int^ aabbMax, int shapeType, void^ userPtr, int collisionFilterGroup, int collisionFilterMask, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBroadphaseInterface_destroyProxy(void* obj, btBroadphaseProxy^ proxy, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBroadphaseInterface_getAabb(void* obj, btBroadphaseProxy^ proxy, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBroadphaseInterface_getBroadphaseAabb(void* obj, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBroadphaseInterface_getOverlappingPairCache(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBroadphaseInterface_getOverlappingPairCache(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBroadphaseInterface_printStats(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBroadphaseInterface_rayTest(void* obj, int^ rayFrom, int^ rayTo, btBroadphaseRayCallback^ rayCallback, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBroadphaseInterface_resetPool(void* obj, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBroadphaseInterface_setAabb(void* obj, btBroadphaseProxy^ proxy, int^ aabbMin, int^ aabbMax, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBroadphasePairSortPredicate_operator()(void* obj, int^ a, int^ b);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btBroadphaseRayCallback_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btBulletSerializedArrays_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btBvhTree_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBvhTree__build_sub_tree(void* obj, GIM_BVH_DATA_ARRAY^ primitive_boxes, int startIndex, int endIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBvhTree__calc_splitting_axis(void* obj, GIM_BVH_DATA_ARRAY^ primitive_boxes, int startIndex, int endIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBvhTree__sort_and_calc_splitting_index(void* obj, GIM_BVH_DATA_ARRAY^ primitive_boxes, int startIndex, int endIndex, int splitAxis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btBvhTree_build_tree(void* obj, GIM_BVH_DATA_ARRAY^ primitive_boxes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCapsuleShapeX_new(int radius, int height);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCapsuleShapeX_getName(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCapsuleShapeZ_new(int radius, int height);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCapsuleShapeZ_getName(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCharacterControllerInterface_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCharacterControllerInterface_canJump(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCharacterControllerInterface_jump(void* obj, int^ dir);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCharacterControllerInterface_onGround(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCharacterControllerInterface_playerStep(void* obj, btCollisionWorld^ collisionWorld, int dt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCharacterControllerInterface_preStep(void* obj, btCollisionWorld^ collisionWorld);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCharacterControllerInterface_reset(void* obj, btCollisionWorld^ collisionWorld);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCharacterControllerInterface_setUpInterpolate(void* obj, bool value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCharacterControllerInterface_setVelocityForTimeInterval(void* obj, int^ velocity, int timeInterval);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCharacterControllerInterface_setWalkDirection(void* obj, int^ walkDirection);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCharacterControllerInterface_warp(void* obj, int^ origin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btClock_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btClock_new(btClock^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btClock_getTimeMicroseconds(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btClock_getTimeMilliseconds(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btClock_getTimeNanoseconds(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btClock_getTimeSeconds(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btClock_operator=(void* obj, btClock^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btClock_reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCollisionAlgorithm_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCollisionAlgorithm_new(btCollisionAlgorithmConstructionInfo^ ci);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionAlgorithm_calculateTimeOfImpact(void* obj, btCollisionObject^ body0, btCollisionObject^ body1, btDispatcherInfo^ dispatchInfo, btManifoldResult^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionAlgorithm_processCollision(void* obj, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, btDispatcherInfo^ dispatchInfo, btManifoldResult^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCollisionAlgorithmConstructionInfo_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCollisionAlgorithmConstructionInfo_new(btDispatcher^ dispatcher, int temp);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCollisionAlgorithmCreateFunc_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionAlgorithmCreateFunc_CreateCollisionAlgorithm(void* obj, btCollisionAlgorithmConstructionInfo^ , btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionConfiguration_getClosestPointsAlgorithmCreateFunc(void* obj, int proxyType0, int proxyType1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionConfiguration_getCollisionAlgorithmCreateFunc(void* obj, int proxyType0, int proxyType1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionConfiguration_getCollisionAlgorithmPool(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionConfiguration_getPersistentManifoldPool(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCollisionDispatcher_new(btCollisionConfiguration^ collisionConfiguration);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_allocateCollisionAlgorithm(void* obj, int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_clearManifold(void* obj, int^ manifold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_defaultNearCallback(void* obj, int^ collisionPair, btCollisionDispatcher^ dispatcher, int^ dispatchInfo);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_dispatchAllCollisionPairs(void* obj, btOverlappingPairCache^ pairCache, int^ dispatchInfo, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_findAlgorithm(void* obj, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ sharedManifold, int queryType);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_freeCollisionAlgorithm(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_getCollisionConfiguration(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_getCollisionConfiguration(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_getDispatcherFlags(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_getInternalManifoldPointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_getInternalManifoldPool(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_getInternalManifoldPool(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_getManifoldByIndexInternal(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_getManifoldByIndexInternal(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_getNearCallback(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_getNewManifold(void* obj, btCollisionObject^ b0, btCollisionObject^ b1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_getNumManifolds(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_needsCollision(void* obj, btCollisionObject^ body0, btCollisionObject^ body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_needsResponse(void* obj, btCollisionObject^ body0, btCollisionObject^ body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_registerClosestPointsCreateFunc(void* obj, int proxyType0, int proxyType1, btCollisionAlgorithmCreateFunc^ createFunc);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_registerCollisionCreateFunc(void* obj, int proxyType0, int proxyType1, btCollisionAlgorithmCreateFunc^ createFunc);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_releaseManifold(void* obj, int^ manifold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_setCollisionConfiguration(void* obj, btCollisionConfiguration^ config);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_setDispatcherFlags(void* obj, int flags);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcher_setNearCallback(void* obj, btNearCallback nearCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCollisionDispatcherMt_new(int^ config, int grainSize);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcherMt_dispatchAllCollisionPairs(void* obj, int^ pairCache, int^ info, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcherMt_getNewManifold(void* obj, int^ body0, int^ body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionDispatcherMt_releaseManifold(void* obj, int^ manifold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCollisionObjectWrapper_new(btCollisionObjectWrapper^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCollisionObjectWrapper_new(btCollisionObjectWrapper^ parent, btCollisionShape^ shape, btCollisionObject^ collisionObject, btTransform^ worldTransform, int partId, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionObjectWrapper_operator delete(void* obj, void^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionObjectWrapper_operator new(void* obj, int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionObjectWrapper_operator=(void* obj, btCollisionObjectWrapper^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCollisionWorld_new(int^ dispatcher, btBroadphaseInterface^ broadphasePairCache, btCollisionConfiguration^ collisionConfiguration);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_addCollisionObject(void* obj, btCollisionObject^ collisionObject, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_computeOverlappingPairs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_contactPairTest(void* obj, btCollisionObject^ colObjA, btCollisionObject^ colObjB, ContactResultCallback^ resultCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_contactTest(void* obj, btCollisionObject^ colObj, ContactResultCallback^ resultCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_convexSweepTest(void* obj, btConvexShape^ castShape, int^ from, int^ to, ConvexResultCallback^ resultCallback, int allowedCcdPenetration);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_debugDrawObject(void* obj, int^ worldTransform, btCollisionShape^ shape, int^ color);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_debugDrawWorld(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_getBroadphase(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_getBroadphase(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_getCollisionObjectArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_getCollisionObjectArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_getDebugDrawer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_getDispatcher(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_getDispatcher(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_getDispatchInfo(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_getDispatchInfo(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_getForceUpdateAllAabbs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_getNumCollisionObjects(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_getPairCache(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_objectQuerySingle(void* obj, btConvexShape^ castShape, int^ rayFromTrans, int^ rayToTrans, btCollisionObject^ collisionObject, btCollisionShape^ collisionShape, int^ colObjWorldTransform, ConvexResultCallback^ resultCallback, int allowedPenetration);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_objectQuerySingleInternal(void* obj, btConvexShape^ castShape, int^ convexFromTrans, int^ convexToTrans, btCollisionObjectWrapper^ colObjWrap, ConvexResultCallback^ resultCallback, int allowedPenetration);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_performDiscreteCollisionDetection(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_rayTest(void* obj, int^ rayFromWorld, int^ rayToWorld, RayResultCallback^ resultCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_rayTestSingle(void* obj, int^ rayFromTrans, int^ rayToTrans, btCollisionObject^ collisionObject, btCollisionShape^ collisionShape, int^ colObjWorldTransform, RayResultCallback^ resultCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_rayTestSingleInternal(void* obj, int^ rayFromTrans, int^ rayToTrans, btCollisionObjectWrapper^ collisionObjectWrap, RayResultCallback^ resultCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_removeCollisionObject(void* obj, btCollisionObject^ collisionObject);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_serialize(void* obj, btSerializer^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_serializeCollisionObjects(void* obj, btSerializer^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_setBroadphase(void* obj, btBroadphaseInterface^ pairCache);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_setDebugDrawer(void* obj, btIDebugDraw^ debugDrawer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_setForceUpdateAllAabbs(void* obj, bool forceUpdateAllAabbs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_updateAabbs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorld_updateSingleAabb(void* obj, btCollisionObject^ colObj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCollisionWorldImporter_new(btCollisionWorld^ world);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_convertAllObjects(void* obj, btBulletSerializedArrays^ arrays);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_convertCollisionShape(void* obj, btCollisionShapeData^ shapeData);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createBoxShape(void* obj, int^ halfExtents);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createBvhTriangleMeshShape(void* obj, btStridingMeshInterface^ trimesh, btOptimizedBvh^ bvh);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createCapsuleShapeX(void* obj, int radius, int height);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createCapsuleShapeY(void* obj, int radius, int height);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createCapsuleShapeZ(void* obj, int radius, int height);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createCollisionObject(void* obj, int^ startTransform, btCollisionShape^ shape, char^ bodyName);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createCompoundShape(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createConeShapeX(void* obj, int radius, int height);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createConeShapeY(void* obj, int radius, int height);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createConeShapeZ(void* obj, int radius, int height);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createConvexHullShape(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createConvexTriangleMeshShape(void* obj, btStridingMeshInterface^ trimesh);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createCylinderShapeX(void* obj, int radius, int height);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createCylinderShapeY(void* obj, int radius, int height);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createCylinderShapeZ(void* obj, int radius, int height);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createMeshInterface(void* obj, btStridingMeshInterfaceData^ meshData);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createMultiSphereShape(void* obj, int^ positions, int^ radi, int numSpheres);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createOptimizedBvh(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createPlaneShape(void* obj, int^ planeNormal, int planeConstant);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createScaledTrangleMeshShape(void* obj, btBvhTriangleMeshShape^ meshShape, int^ localScalingbtBvhTriangleMeshShape);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createSphereShape(void* obj, int radius);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createStridingMeshInterfaceData(void* obj, btStridingMeshInterfaceData^ interfaceData);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createTriangleInfoMap(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_createTriangleMeshContainer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_deleteAllData(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_duplicateName(void* obj, char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_getBvhByIndex(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_getCollisionObjectByName(void* obj, char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_getCollisionShapeByIndex(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_getCollisionShapeByName(void* obj, char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_getNameForPointer(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_getNumBvhs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_getNumCollisionShapes(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_getNumRigidBodies(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_getNumTriangleInfoMaps(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_getRigidBodyByIndex(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_getTriangleInfoMapByIndex(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_getVerboseMode(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCollisionWorldImporter_setVerboseMode(void* obj, int verboseMode);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCompoundCollisionAlgorithm_new(btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, bool isSwapped);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCompoundCollisionAlgorithm_calculateTimeOfImpact(void* obj, btCollisionObject^ body0, btCollisionObject^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCompoundCollisionAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCompoundCollisionAlgorithm_getChildAlgorithm(void* obj, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCompoundCollisionAlgorithm_preallocateChildAlgorithms(void* obj, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCompoundCollisionAlgorithm_processCollision(void* obj, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCompoundCollisionAlgorithm_removeChildAlgorithms(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCompoundCompoundCollisionAlgorithm_new(btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, bool isSwapped);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCompoundCompoundCollisionAlgorithm_calculateTimeOfImpact(void* obj, btCollisionObject^ body0, btCollisionObject^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCompoundCompoundCollisionAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCompoundCompoundCollisionAlgorithm_processCollision(void* obj, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCompoundCompoundCollisionAlgorithm_removeChildAlgorithms(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btConeShapeX_new(int radius, int height);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConeShapeX_getAnisotropicRollingFrictionDirection(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConeShapeX_getName(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btConeShapeZ_new(int radius, int height);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConeShapeZ_getAnisotropicRollingFrictionDirection(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConeShapeZ_getName(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btConstraintSetting_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConstraintSolver_allSolved(void* obj, btContactSolverInfo^ , btIDebugDraw^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConstraintSolver_getSolverType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConstraintSolver_prepareSolve(void* obj, int , int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConstraintSolver_reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConstraintSolver_solveGroup(void* obj, btCollisionObject^ bodies, int numBodies, btPersistentManifold^ manifold, int numManifolds, btTypedConstraint^ constraints, int numConstraints, btContactSolverInfo^ info, btIDebugDraw^ debugDrawer, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btConstraintSolverPoolMt_new(int numSolvers);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btConstraintSolverPoolMt_new(btConstraintSolver^ solvers, int numSolvers);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConstraintSolverPoolMt_getAndLockThreadSolver(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConstraintSolverPoolMt_getSolverType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConstraintSolverPoolMt_init(void* obj, btConstraintSolver^ solvers, int numSolvers);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConstraintSolverPoolMt_reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConstraintSolverPoolMt_solveGroup(void* obj, int^ bodies, int numBodies, btPersistentManifold^ manifolds, int numManifolds, btTypedConstraint^ constraints, int numConstraints, int^ info, btIDebugDraw^ debugDrawer, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btContactArray_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btContactArray_merge_contacts_unique(void* obj, btContactArray^ contacts);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btContactSolverInfo_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btContinuousConvexCollision_new(btConvexShape^ shapeA, btConvexShape^ shapeB, int^ simplexSolver, btConvexPenetrationDepthSolver^ penetrationDepthSolver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btContinuousConvexCollision_new(btConvexShape^ shapeA, btStaticPlaneShape^ plane);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btContinuousConvexCollision_calcTimeOfImpact(void* obj, int^ fromA, int^ toA, int^ fromB, int^ toB, CastResult^ result);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btContinuousConvexCollision_computeClosestPoints(void* obj, int^ transA, int^ transB, btPointCollector^ pointCollector);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btConvex2dConvex2dAlgorithm_new(int^ mf, int^ ci, int^ body0Wrap, int^ body1Wrap, int^ simplexSolver, btConvexPenetrationDepthSolver^ pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvex2dConvex2dAlgorithm_calculateTimeOfImpact(void* obj, int^ body0, int^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvex2dConvex2dAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvex2dConvex2dAlgorithm_getManifold(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvex2dConvex2dAlgorithm_processCollision(void* obj, int^ body0Wrap, int^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvex2dConvex2dAlgorithm_setLowLevelOfDetail(void* obj, bool useLowLevel);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexCast_calcTimeOfImpact(void* obj, int^ fromA, int^ toA, int^ fromB, int^ toB, CastResult^ result);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btConvexConvexAlgorithm_new(int^ mf, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, btConvexPenetrationDepthSolver^ pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexConvexAlgorithm_calculateTimeOfImpact(void* obj, btCollisionObject^ body0, btCollisionObject^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexConvexAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexConvexAlgorithm_getManifold(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexConvexAlgorithm_processCollision(void* obj, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexConvexAlgorithm_setLowLevelOfDetail(void* obj, bool useLowLevel);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexHullComputer_compute(void* obj, void^ coords, bool doubleCoords, int stride, int count, float shrink, float shrinkClamp);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexHullComputer_compute(void* obj, float^ coords, int stride, int count, float shrink, float shrinkClamp);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexHullComputer_compute(void* obj, double^ coords, int stride, int count, float shrink, float shrinkClamp);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btConvexInternalAabbCachingShape_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexInternalAabbCachingShape_getAabb(void* obj, int^ t, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexInternalAabbCachingShape_getCachedLocalAabb(void* obj, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexInternalAabbCachingShape_getNonvirtualAabb(void* obj, int^ trans, int^ aabbMin, int^ aabbMax, int margin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexInternalAabbCachingShape_recalcLocalAabb(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexInternalAabbCachingShape_setCachedLocalAabb(void* obj, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexInternalAabbCachingShape_setLocalScaling(void* obj, int^ scaling);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexPenetrationDepthSolver_calcPenDepth(void* obj, int^ simplexSolver, btConvexShape^ convexA, btConvexShape^ convexB, btTransform^ transA, btTransform^ transB, btVector3^ v, btVector3^ pa, btVector3^ pb, btIDebugDraw^ debugDraw);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btConvexPlaneCollisionAlgorithm_new(btPersistentManifold^ mf, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, bool isSwapped, int numPerturbationIterations, int minimumPointsPerturbationThreshold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexPlaneCollisionAlgorithm_calculateTimeOfImpact(void* obj, btCollisionObject^ body0, btCollisionObject^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexPlaneCollisionAlgorithm_collideSingleContact(void* obj, int^ perturbeRot, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexPlaneCollisionAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexPlaneCollisionAlgorithm_processCollision(void* obj, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btConvexSeparatingDistanceUtil_new(float boundingRadiusA, float boundingRadiusB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexSeparatingDistanceUtil_getConservativeSeparatingDistance(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexSeparatingDistanceUtil_initSeparatingDistance(void* obj, btVector3^ separatingVector, float separatingDistance, btTransform^ transA, btTransform^ transB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btConvexSeparatingDistanceUtil_updateSeparatingDistance(void* obj, btTransform^ transA, btTransform^ transB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCpuFeatureUtility_getCpuFeatures(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCPUVertexBufferDescriptor_new(float^ basePointer, int vertexOffset, int vertexStride);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCPUVertexBufferDescriptor_new(float^ basePointer, int vertexOffset, int vertexStride, int normalOffset, int normalStride);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCPUVertexBufferDescriptor_getBasePointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCPUVertexBufferDescriptor_getBufferType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCylinderShapeX_new(int^ halfExtents);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCylinderShapeX_batchedUnitVectorGetSupportingVertexWithoutMargin(void* obj, int^ vectors, int^ supportVerticesOut, int numVectors);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCylinderShapeX_BT_DECLARE_ALIGNED_ALLOCATOR(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCylinderShapeX_getName(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCylinderShapeX_getRadius(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCylinderShapeX_localGetSupportingVertexWithoutMargin(void* obj, int^ vec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btCylinderShapeZ_new(int^ halfExtents);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCylinderShapeZ_batchedUnitVectorGetSupportingVertexWithoutMargin(void* obj, int^ vectors, int^ supportVerticesOut, int numVectors);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCylinderShapeZ_BT_DECLARE_ALIGNED_ALLOCATOR(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCylinderShapeZ_getName(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCylinderShapeZ_getRadius(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btCylinderShapeZ_localGetSupportingVertexWithoutMargin(void* obj, int^ vec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btDantzigSolver_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDantzigSolver_solveMLCP(void* obj, int^ A, int^ b, int^ x, int^ lo, int^ hi, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btDbvt_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_benchmark(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_clear(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_clone(void* obj, btDbvt^ dest, IClone^ iclone);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_collideKDOP(void* obj, btDbvtNode^ root, int^ normals, int^ offsets, int count, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_collideOCL(void* obj, btDbvtNode^ root, int^ normals, int^ offsets, int^ sortaxis, int count, ICollide^ policy, bool fullsort);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_collideTT(void* obj, btDbvtNode^ root0, btDbvtNode^ root1, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_collideTTpersistentStack(void* obj, btDbvtNode^ root0, btDbvtNode^ root1, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_collideTU(void* obj, btDbvtNode^ root, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_collideTV(void* obj, btDbvtNode^ root, btDbvtVolume^ volume, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_collideTVNoStackAlloc(void* obj, btDbvtNode^ root, btDbvtVolume^ volume, int^ stack, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_countLeaves(void* obj, btDbvtNode^ node);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_empty(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_enumLeaves(void* obj, btDbvtNode^ root, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_enumNodes(void* obj, btDbvtNode^ root, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_extractLeaves(void* obj, btDbvtNode^ node, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_insert(void* obj, btDbvtVolume^ box, void^ data);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_maxdepth(void* obj, btDbvtNode^ node);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_optimizeBottomUp(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_optimizeIncremental(void* obj, int passes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_optimizeTopDown(void* obj, int bu_treshold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_rayTest(void* obj, btDbvtNode^ root, int^ rayFrom, int^ rayTo, ICollide^ policy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_rayTestInternal(void* obj, btDbvtNode^ root, int^ rayFrom, int^ rayTo, int^ rayDirectionInverse, uint^ signs, int lambda_max, int^ aabbMin, int^ aabbMax, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_remove(void* obj, btDbvtNode^ leaf);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_update(void* obj, btDbvtNode^ leaf, int lookahead);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_update(void* obj, btDbvtNode^ leaf, btDbvtVolume^ volume);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_update(void* obj, btDbvtNode^ leaf, btDbvtVolume^ volume, int^ velocity, int margin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_update(void* obj, btDbvtNode^ leaf, btDbvtVolume^ volume, int^ velocity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_update(void* obj, btDbvtNode^ leaf, btDbvtVolume^ volume, int margin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvt_write(void* obj, IWriter^ iwriter);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtAabbMm_FromCR(void* obj, int^ c, int r);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtAabbMm_FromMM(void* obj, int^ mi, int^ mx);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtAabbMm_FromPoints(void* obj, int^ pts, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtAabbMm_FromPoints(void* obj, int^ ppts, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btDbvtBroadphase_new(int^ paircache);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_aabbTest(void* obj, int^ aabbMin, int^ aabbMax, int^ callback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_benchmark(void* obj, int^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_calculateOverlappingPairs(void* obj, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_collide(void* obj, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_createProxy(void* obj, int^ aabbMin, int^ aabbMax, int shapeType, void^ userPtr, int collisionFilterGroup, int collisionFilterMask, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_destroyProxy(void* obj, int^ proxy, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_getAabb(void* obj, int^ proxy, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_getBroadphaseAabb(void* obj, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_getOverlappingPairCache(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_getOverlappingPairCache(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_getVelocityPrediction(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_optimize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_performDeferredRemoval(void* obj, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_printStats(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_rayTest(void* obj, int^ rayFrom, int^ rayTo, int^ rayCallback, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_resetPool(void* obj, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_setAabb(void* obj, int^ proxy, int^ aabbMin, int^ aabbMax, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_setAabbForceUpdate(void* obj, int^ absproxy, int^ aabbMin, int^ aabbMax, int^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDbvtBroadphase_setVelocityPrediction(void* obj, int prediction);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btDbvtProxy_new(int^ aabbMin, int^ aabbMax, void^ userPtr, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btDefaultCollisionConfiguration_new(btDefaultCollisionConstructionInfo^ constructionInfo);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultCollisionConfiguration_getClosestPointsAlgorithmCreateFunc(void* obj, int proxyType0, int proxyType1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultCollisionConfiguration_getCollisionAlgorithmCreateFunc(void* obj, int proxyType0, int proxyType1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultCollisionConfiguration_getCollisionAlgorithmPool(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultCollisionConfiguration_getPersistentManifoldPool(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultCollisionConfiguration_setConvexConvexMultipointIterations(void* obj, int numPerturbationIterations, int minimumPointsPerturbationThreshold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultCollisionConfiguration_setPlaneConvexMultipointIterations(void* obj, int numPerturbationIterations, int minimumPointsPerturbationThreshold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btDefaultCollisionConstructionInfo_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btDefaultMotionState_new(btTransform^ startTrans, btTransform^ centerOfMassOffset);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultMotionState_getWorldTransform(void* obj, btTransform^ centerOfMassWorldTrans);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultMotionState_operator delete(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultMotionState_operator delete(void* obj, void^ , void^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultMotionState_operator delete[](void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultMotionState_operator delete[](void* obj, void^ , void^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultMotionState_operator new(void* obj, int sizeInBytes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultMotionState_operator new(void* obj, int size_t, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultMotionState_operator new[](void* obj, int sizeInBytes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultMotionState_operator new[](void* obj, int size_t, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultMotionState_setWorldTransform(void* obj, btTransform^ centerOfMassWorldTrans);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btDefaultSerializer_new(int totalSize, byte^ buffer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_allocate(void* obj, int size, int numElements);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_finalizeChunk(void* obj, btChunk^ chunk, char^ structType, int chunkCode, void^ oldPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_findNameForPointer(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_findPointer(void* obj, void^ oldPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_finishSerialization(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_getBufferPointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_getChunk(void* obj, int chunkIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_getCurrentBufferSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_getMemoryDna(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_getMemoryDnaSizeInBytes(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_getNumChunks(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_getReverseType(void* obj, char^ type);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_getSerializationFlags(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_getUniquePointer(void* obj, void^ oldPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_initDNA(void* obj, char^ bdnaOrg, int dnalen);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_insertHeader(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_internalAlloc(void* obj, int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_registerNameForPointer(void* obj, void^ ptr, char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_serializeName(void* obj, char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_setSerializationFlags(void* obj, int flags);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_startSerialization(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_writeDNA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSerializer_writeHeader(void* obj, byte^ buffer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btDefaultSoftBodySolver_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSoftBodySolver_checkInitialized(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSoftBodySolver_copyBackToSoftBodies(void* obj, bool bMove);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSoftBodySolver_copySoftBodyToVertexBuffer(void* obj, int^ softBody, btVertexBufferDescriptor^ vertexBuffer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSoftBodySolver_getSolverType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSoftBodySolver_optimize(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSoftBodySolver_predictMotion(void* obj, float solverdt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSoftBodySolver_processCollision(void* obj, int^ , btCollisionObjectWrapper^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSoftBodySolver_processCollision(void* obj, int^ , int^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSoftBodySolver_solveConstraints(void* obj, float solverdt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultSoftBodySolver_updateSoftBodies(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btDefaultVehicleRaycaster_new(btDynamicsWorld^ world);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDefaultVehicleRaycaster_castRay(void* obj, int^ from, int^ to, btVehicleRaycasterResult^ result);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDiscreteCollisionDetectorInterface_getClosestPoints(void* obj, ClosestPointInput^ input, Result^ output, btIDebugDraw^ debugDraw, bool swapResults);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDispatcher_allocateCollisionAlgorithm(void* obj, int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDispatcher_clearManifold(void* obj, btPersistentManifold^ manifold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDispatcher_dispatchAllCollisionPairs(void* obj, btOverlappingPairCache^ pairCache, btDispatcherInfo^ dispatchInfo, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDispatcher_findAlgorithm(void* obj, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, btPersistentManifold^ sharedManifold, ebtDispatcherQueryType queryType);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDispatcher_freeCollisionAlgorithm(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDispatcher_getInternalManifoldPointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDispatcher_getInternalManifoldPool(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDispatcher_getInternalManifoldPool(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDispatcher_getManifoldByIndexInternal(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDispatcher_getNewManifold(void* obj, btCollisionObject^ b0, btCollisionObject^ b1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDispatcher_getNumManifolds(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDispatcher_needsCollision(void* obj, btCollisionObject^ body0, btCollisionObject^ body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDispatcher_needsResponse(void* obj, btCollisionObject^ body0, btCollisionObject^ body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDispatcher_releaseManifold(void* obj, btPersistentManifold^ manifold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btDispatcherInfo_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btDynamicsWorld_new(int^ dispatcher, int^ broadphase, int^ collisionConfiguration);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_addAction(void* obj, btActionInterface^ action);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_addCharacter(void* obj, btActionInterface^ character);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_addConstraint(void* obj, btTypedConstraint^ constraint, bool disableCollisionsBetweenLinkedBodies);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_addRigidBody(void* obj, int^ body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_addRigidBody(void* obj, int^ body, int group, int mask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_addVehicle(void* obj, btActionInterface^ vehicle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_clearForces(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_debugDrawWorld(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_getConstraint(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_getConstraint(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_getConstraintSolver(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_getGravity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_getNumConstraints(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_getSolverInfo(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_getSolverInfo(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_getWorldType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_getWorldUserInfo(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_removeAction(void* obj, btActionInterface^ action);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_removeCharacter(void* obj, btActionInterface^ character);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_removeConstraint(void* obj, btTypedConstraint^ constraint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_removeRigidBody(void* obj, int^ body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_removeVehicle(void* obj, btActionInterface^ vehicle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_setConstraintSolver(void* obj, btConstraintSolver^ solver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_setGravity(void* obj, int^ gravity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_setInternalTickCallback(void* obj, btInternalTickCallback cb, void^ worldUserInfo, bool isPreTick);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_setWorldUserInfo(void* obj, void^ worldUserInfo);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_stepSimulation(void* obj, int timeStep, int maxSubSteps, int fixedTimeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btDynamicsWorld_synchronizeMotionStates(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btEigen_mulPQ(void* obj, int^ a, int c, int s, int p, int q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btEigen_mulTPQ(void* obj, int^ a, int c, int s, int p, int q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btEigen_system(void* obj, int^ a, int^ vectors, int^ values);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btEmptyAlgorithm_new(btCollisionAlgorithmConstructionInfo^ ci);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btEmptyAlgorithm_calculateTimeOfImpact(void* obj, btCollisionObject^ body0, btCollisionObject^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btEmptyAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btEmptyAlgorithm_processCollision(void* obj, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGearConstraint_new(int^ rbA, int^ rbB, int^ axisInA, int^ axisInB, int ratio);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGearConstraint_calculateSerializeBufferSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGearConstraint_getAxisA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGearConstraint_getAxisB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGearConstraint_getInfo1(void* obj, int^ info);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGearConstraint_getInfo2(void* obj, int^ info);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGearConstraint_getParam(void* obj, int num, int axis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGearConstraint_getRatio(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGearConstraint_serialize(void* obj, void^ dataBuffer, int^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGearConstraint_setAxisA(void* obj, int^ axisA);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGearConstraint_setAxisB(void* obj, int^ axisB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGearConstraint_setParam(void* obj, int num, int value, int axis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGearConstraint_setRatio(void* obj, int ratio);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGEN_Link_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGEN_Link_new(btGEN_Link^ next, btGEN_Link^ prev);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGEN_Link_getNext(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGEN_Link_getPrev(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGEN_Link_insertAfter(void* obj, btGEN_Link^ link);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGEN_Link_insertBefore(void* obj, btGEN_Link^ link);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGEN_Link_isHead(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGEN_Link_isTail(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGEN_Link_remove(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGEN_List_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGEN_List_addHead(void* obj, btGEN_Link^ link);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGEN_List_addTail(void* obj, btGEN_Link^ link);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGEN_List_getHead(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGEN_List_getTail(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGenericMemoryPool_new(int element_size, int element_count);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericMemoryPool_allocate(void* obj, int size_bytes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericMemoryPool_allocate_from_free_nodes(void* obj, int num_elements);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericMemoryPool_allocate_from_pool(void* obj, int num_elements);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericMemoryPool_end_pool(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericMemoryPool_freeMemory(void* obj, void^ pointer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericMemoryPool_gem_element_size(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericMemoryPool_get_allocated_count(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericMemoryPool_get_element_data(void* obj, int element_index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericMemoryPool_get_free_positions_count(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericMemoryPool_get_max_element_count(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericMemoryPool_get_pool_capacity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericMemoryPool_init_pool(void* obj, int element_size, int element_count);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGenericPoolAllocator_new(int pool_element_size, int pool_element_count);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericPoolAllocator_allocate(void* obj, int size_bytes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericPoolAllocator_failback_alloc(void* obj, int size_bytes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericPoolAllocator_failback_free(void* obj, void^ pointer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericPoolAllocator_freeMemory(void* obj, void^ pointer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericPoolAllocator_get_pool_capacity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGenericPoolAllocator_push_new_pool(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGeometryUtil_areVerticesBehindPlane(void* obj, btVector3^ planeNormal, btAlignedObjectArray^ vertices, float margin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGeometryUtil_getPlaneEquationsFromVertices(void* obj, btAlignedObjectArray^ vertices, btAlignedObjectArray^ planeEquationsOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGeometryUtil_getVerticesFromPlaneEquations(void* obj, btAlignedObjectArray^ planeEquations, btAlignedObjectArray^ verticesOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGeometryUtil_isInside(void* obj, btAlignedObjectArray^ vertices, btVector3^ planeNormal, float margin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGeometryUtil_isPointInsidePlanes(void* obj, btAlignedObjectArray^ planeEquations, btVector3^ point, float margin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGhostPairCallback_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGhostPairCallback_addOverlappingPair(void* obj, btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGhostPairCallback_removeOverlappingPair(void* obj, btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGhostPairCallback_removeOverlappingPairsContainingProxy(void* obj, btBroadphaseProxy^ , btDispatcher^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGImpactBvh_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGImpactBvh_new(btPrimitiveManagerBase^ primitive_manager);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactBvh_boxQuery(void* obj, int^ box, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactBvh_refit(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGImpactCollisionAlgorithm_new(int^ ci, int^ body0Wrap, int^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_calculateTimeOfImpact(void* obj, int^ body0, int^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_collide_gjk_triangles(void* obj, int^ body0Wrap, int^ body1Wrap, btGImpactMeshShapePart^ shape0, btGImpactMeshShapePart^ shape1, int^ pairs, int pair_count);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_collide_sat_triangles(void* obj, int^ body0Wrap, int^ body1Wrap, btGImpactMeshShapePart^ shape0, btGImpactMeshShapePart^ shape1, int^ pairs, int pair_count);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_convex_vs_convex_collision(void* obj, int^ body0Wrap, int^ body1Wrap, int^ shape0, int^ shape1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_getFace0(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_getFace1(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_getPart0(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_getPart1(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_gimpact_vs_compoundshape(void* obj, int^ body0Wrap, int^ body1Wrap, btGImpactShapeInterface^ shape0, int^ shape1, bool swapped);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_gimpact_vs_concave(void* obj, int^ body0Wrap, int^ body1Wrap, btGImpactShapeInterface^ shape0, int^ shape1, bool swapped);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_gimpact_vs_gimpact(void* obj, int^ body0Wrap, int^ body1Wrap, btGImpactShapeInterface^ shape0, btGImpactShapeInterface^ shape1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_gimpact_vs_gimpact_find_pairs(void* obj, int^ trans0, int^ trans1, btGImpactShapeInterface^ shape0, btGImpactShapeInterface^ shape1, btPairSet^ pairset);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_gimpact_vs_shape(void* obj, int^ body0Wrap, int^ body1Wrap, btGImpactShapeInterface^ shape0, int^ shape1, bool swapped);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_gimpact_vs_shape_find_pairs(void* obj, int^ trans0, int^ trans1, btGImpactShapeInterface^ shape0, int^ shape1, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_gimpacttrimeshpart_vs_plane_collision(void* obj, int^ body0Wrap, int^ body1Wrap, btGImpactMeshShapePart^ shape0, int^ shape1, bool swapped);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_internalGetResultOut(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_processCollision(void* obj, int^ body0Wrap, int^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_registerAlgorithm(void* obj, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_setFace0(void* obj, int value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_setFace1(void* obj, int value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_setPart0(void* obj, int value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_setPart1(void* obj, int value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCollisionAlgorithm_shape_vs_shape_collision(void* obj, int^ body0, int^ body1, int^ shape0, int^ shape1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGImpactCompoundShape_new(bool children_has_transform);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCompoundShape_childrenHasTransform(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCompoundShape_getGImpactShapeType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCompoundShape_getName(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactCompoundShape_getPrimitiveManager(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGImpactMeshShape_new(int^ meshInterface);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_buildMeshParts(void* obj, int^ meshInterface);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_calcLocalAABB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_calculateLocalInertia(void* obj, int mass, int^ inertia);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_calculateSerializeBufferSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_childrenHasTransform(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getBulletTetrahedron(void* obj, int prim_index, btTetrahedronShapeEx^ tetrahedron);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getBulletTriangle(void* obj, int prim_index, btTriangleShapeEx^ triangle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getChildAabb(void* obj, int child_index, int^ t, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getChildShape(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getChildShape(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getChildTransform(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getGImpactShapeType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getMeshInterface(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getMeshInterface(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getMeshPart(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getMeshPart(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getMeshPartCount(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getName(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getNumChildShapes(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_getPrimitiveManager(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_lockChildShapes(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_needsRetrieveTetrahedrons(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_needsRetrieveTriangles(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_postUpdate(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_processAllTriangles(void* obj, int^ callback, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_processAllTrianglesRay(void* obj, int^ callback, int^ rayFrom, int^ rayTo);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_rayTest(void* obj, int^ rayFrom, int^ rayTo, int^ resultCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_serialize(void* obj, void^ dataBuffer, int^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_setChildTransform(void* obj, int index, int^ transform);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_setLocalScaling(void* obj, int^ scaling);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_setMargin(void* obj, int margin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShape_unlockChildShapes(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGImpactMeshShapePart_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGImpactMeshShapePart_new(int^ meshInterface, int part);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_childrenHasTransform(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_getBulletTetrahedron(void* obj, int prim_index, btTetrahedronShapeEx^ tetrahedron);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_getBulletTriangle(void* obj, int prim_index, btTriangleShapeEx^ triangle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_getChildShape(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_getChildShape(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_getChildTransform(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_getGImpactShapeType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_getName(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_getNumChildShapes(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_getPrimitiveManager(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_lockChildShapes(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_needsRetrieveTetrahedrons(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_needsRetrieveTriangles(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_processAllTrianglesRay(void* obj, int^ callback, int^ rayFrom, int^ rayTo);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_setChildTransform(void* obj, int index, int^ transform);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactMeshShapePart_unlockChildShapes(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGImpactQuantizedBvh_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGImpactQuantizedBvh_new(btPrimitiveManagerBase^ primitive_manager);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactQuantizedBvh_boxQuery(void* obj, int^ box, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactQuantizedBvh_refit(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGImpactShapeInterface_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_calcLocalAABB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_childrenHasTransform(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_getBulletTetrahedron(void* obj, int prim_index, btTetrahedronShapeEx^ tetrahedron);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_getBulletTriangle(void* obj, int prim_index, btTriangleShapeEx^ triangle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_getChildShape(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_getChildTransform(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_getNumChildShapes(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_lockChildShapes(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_needsRetrieveTetrahedrons(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_needsRetrieveTriangles(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_processAllTriangles(void* obj, int^ callback, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_processAllTrianglesRay(void* obj, int^ , int^ , int^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_rayTest(void* obj, int^ rayFrom, int^ rayTo, int^ resultCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_setChildTransform(void* obj, int index, int^ transform);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGImpactShapeInterface_unlockChildShapes(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGjkCollisionDescription_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGjkConvexCast_new(btConvexShape^ convexA, btConvexShape^ convexB, int^ simplexSolver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkConvexCast_calcTimeOfImpact(void* obj, int^ fromA, int^ toA, int^ fromB, int^ toB, CastResult^ result);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGjkEpaPenetrationDepthSolver_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkEpaPenetrationDepthSolver_calcPenDepth(void* obj, int^ simplexSolver, btConvexShape^ pConvexA, btConvexShape^ pConvexB, btTransform^ transformA, btTransform^ transformB, btVector3^ v, btVector3^ wWitnessOnA, btVector3^ wWitnessOnB, btIDebugDraw^ debugDraw);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkEpaSolver2_Distance(void* obj, int^ shape0, int^ wtrs0, int^ shape1, int^ wtrs1, int^ guess, sResults^ results);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkEpaSolver2_Penetration(void* obj, int^ shape0, int^ wtrs0, int^ shape1, int^ wtrs1, int^ guess, sResults^ results, bool usemargins);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkEpaSolver2_SignedDistance(void* obj, int^ position, int margin, int^ shape, int^ wtrs, sResults^ results);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkEpaSolver2_SignedDistance(void* obj, int^ shape0, int^ wtrs0, int^ shape1, int^ wtrs1, int^ guess, sResults^ results);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkEpaSolver2_StackSizeRequirement(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGjkPairDetector_new(btConvexShape^ objectA, btConvexShape^ objectB, int^ simplexSolver, btConvexPenetrationDepthSolver^ penetrationDepthSolver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btGjkPairDetector_new(btConvexShape^ objectA, btConvexShape^ objectB, int shapeTypeA, int shapeTypeB, int marginA, int marginB, int^ simplexSolver, btConvexPenetrationDepthSolver^ penetrationDepthSolver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkPairDetector_getCachedSeparatingAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkPairDetector_getCachedSeparatingDistance(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkPairDetector_getClosestPoints(void* obj, ClosestPointInput^ input, Result^ output, btIDebugDraw^ debugDraw, bool swapResults);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkPairDetector_getClosestPointsNonVirtual(void* obj, ClosestPointInput^ input, Result^ output, btIDebugDraw^ debugDraw);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkPairDetector_setCachedSeperatingAxis(void* obj, int^ seperatingAxis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkPairDetector_setIgnoreMargin(void* obj, bool ignoreMargin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkPairDetector_setMinkowskiA(void* obj, btConvexShape^ minkA);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkPairDetector_setMinkowskiB(void* obj, btConvexShape^ minkB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btGjkPairDetector_setPenetrationDepthSolver(void* obj, btConvexPenetrationDepthSolver^ penetrationDepthSolver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btHashedSimplePairCache_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashedSimplePairCache_addOverlappingPair(void* obj, int indexA, int indexB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashedSimplePairCache_findPair(void* obj, int indexA, int indexB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashedSimplePairCache_GetCount(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashedSimplePairCache_getNumOverlappingPairs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashedSimplePairCache_getOverlappingPairArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashedSimplePairCache_getOverlappingPairArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashedSimplePairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashedSimplePairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashedSimplePairCache_growTables(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashedSimplePairCache_internalAddPair(void* obj, int indexA, int indexB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashedSimplePairCache_removeAllPairs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashedSimplePairCache_removeOverlappingPair(void* obj, int indexA, int indexB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btHashInt_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btHashInt_new(int uid);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashInt_equals(void* obj, btHashInt^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashInt_getHash(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashInt_getUid1(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashInt_setUid1(void* obj, int uid);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btHashKey_new(int uid);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashKey_equals(void* obj, btHashKey^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashKey_getHash(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashKey_getUid1(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btHashKeyPtr_new(int uid);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashKeyPtr_equals(void* obj, btHashKeyPtr^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashKeyPtr_getHash(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashKeyPtr_getUid1(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashMap_clear(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashMap_find(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashMap_find(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashMap_findIndex(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashMap_getAtIndex(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashMap_getAtIndex(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashMap_getKeyAtIndex(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashMap_getKeyAtIndex(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashMap_growTables(void* obj, [unexposed type]^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashMap_insert(void* obj, [unexposed type]^ key, [unexposed type]^ value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashMap_operator[](void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashMap_operator[](void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashMap_remove(void* obj, [unexposed type]^ key);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashMap_size(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btHashPtr_new(void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashPtr_equals(void* obj, btHashPtr^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashPtr_getHash(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashPtr_getPointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btHashString_new(char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashString_equals(void* obj, btHashString^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashString_getHash(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btHashString_portableStringCompare(void* obj, char^ src, char^ dst);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_clearLines(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_draw3dText(void* obj, btVector3^ location, char^ textString);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawAabb(void* obj, btVector3^ from, btVector3^ to, btVector3^ color);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawArc(void* obj, btVector3^ center, btVector3^ normal, btVector3^ axis, float radiusA, float radiusB, float minAngle, float maxAngle, btVector3^ color, bool drawSect, float stepDegrees);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawBox(void* obj, btVector3^ bbMin, btVector3^ bbMax, btVector3^ color);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawBox(void* obj, btVector3^ bbMin, btVector3^ bbMax, btTransform^ trans, btVector3^ color);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawCapsule(void* obj, float radius, float halfHeight, int upAxis, btTransform^ transform, btVector3^ color);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawCone(void* obj, float radius, float height, int upAxis, btTransform^ transform, btVector3^ color);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawContactPoint(void* obj, btVector3^ PointOnB, btVector3^ normalOnB, float distance, int lifeTime, btVector3^ color);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawCylinder(void* obj, float radius, float halfHeight, int upAxis, btTransform^ transform, btVector3^ color);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawLine(void* obj, btVector3^ from, btVector3^ to, btVector3^ color);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawLine(void* obj, btVector3^ from, btVector3^ to, btVector3^ fromColor, btVector3^ toColor);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawPlane(void* obj, btVector3^ planeNormal, float planeConst, btTransform^ transform, btVector3^ color);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawSphere(void* obj, float radius, btTransform^ transform, btVector3^ color);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawSphere(void* obj, btVector3^ p, float radius, btVector3^ color);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawSpherePatch(void* obj, btVector3^ center, btVector3^ up, btVector3^ axis, float radius, float minTh, float maxTh, float minPs, float maxPs, btVector3^ color, float stepDegrees, bool drawCenter);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawTransform(void* obj, btTransform^ transform, float orthoLen);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawTriangle(void* obj, btVector3^ v0, btVector3^ v1, btVector3^ v2, btVector3^ , btVector3^ , btVector3^ , btVector3^ color, float alpha);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_drawTriangle(void* obj, btVector3^ v0, btVector3^ v1, btVector3^ v2, btVector3^ color, float );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_flushLines(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_getDebugMode(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_getDefaultColors(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_reportErrorWarning(void* obj, char^ warningString);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_setDebugMode(void* obj, int debugMode);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIDebugDraw_setDefaultColors(void* obj, DefaultColors^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btInfMaskConverter_new(int _mask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btInternalTriangleIndexCallback_internalProcessTriangleIndex(void* obj, int^ triangle, int partId, int triangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIntSortPredicate_operator()(void* obj, int^ a, int^ b);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btIParallelForBody_forLoop(void* obj, int iBegin, int iEnd);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btITaskScheduler_new(char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btITaskScheduler_activate(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btITaskScheduler_deactivate(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btITaskScheduler_getMaxNumThreads(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btITaskScheduler_getName(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btITaskScheduler_getNumThreads(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btITaskScheduler_parallelFor(void* obj, int iBegin, int iEnd, int grainSize, btIParallelForBody^ body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btITaskScheduler_setNumThreads(void* obj, int numThreads);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btLemkeAlgorithm_new(int^ M_, int^ q_, int^ DEBUGLEVEL_);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btLemkeAlgorithm_findLexicographicMinimum(void* obj, int^ A, int^ pivotColIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btLemkeAlgorithm_GaussJordanEliminationStep(void* obj, int^ A, int pivotRowIndex, int pivotColumnIndex, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btLemkeAlgorithm_getInfo(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btLemkeAlgorithm_getSteps(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btLemkeAlgorithm_greaterZero(void* obj, int^ vector);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btLemkeAlgorithm_LexicographicPositive(void* obj, int^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btLemkeAlgorithm_setSystem(void* obj, int^ M_, int^ q_);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btLemkeAlgorithm_solve(void* obj, uint maxloops);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btLemkeAlgorithm_validBasis(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btLemkeSolver_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btLemkeSolver_solveMLCP(void* obj, int^ A, int^ b, int^ x, int^ lo, int^ hi, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btManifoldPoint_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btManifoldPoint_new(int^ pointA, int^ pointB, int^ normal, int distance);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldPoint_getAppliedImpulse(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldPoint_getDistance(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldPoint_getLifeTime(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldPoint_getPositionWorldOnA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldPoint_getPositionWorldOnB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldPoint_setDistance(void* obj, int dist);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btManifoldResult_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btManifoldResult_new(btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldResult_addContactPoint(void* obj, int^ normalOnBInWorld, int^ pointInWorld, int depth);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldResult_calculateCombinedContactDamping(void* obj, btCollisionObject^ body0, btCollisionObject^ body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldResult_calculateCombinedContactStiffness(void* obj, btCollisionObject^ body0, btCollisionObject^ body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldResult_calculateCombinedFriction(void* obj, btCollisionObject^ body0, btCollisionObject^ body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldResult_calculateCombinedRestitution(void* obj, btCollisionObject^ body0, btCollisionObject^ body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldResult_calculateCombinedRollingFriction(void* obj, btCollisionObject^ body0, btCollisionObject^ body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldResult_calculateCombinedSpinningFriction(void* obj, btCollisionObject^ body0, btCollisionObject^ body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldResult_getPersistentManifold(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldResult_getPersistentManifold(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldResult_setPersistentManifold(void* obj, int^ manifoldPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldResult_setShapeIdentifiersA(void* obj, int partId0, int index0);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btManifoldResult_setShapeIdentifiersB(void* obj, int partId1, int index1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMaterial_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMaterial_new(int fric, int rest);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMatrix3x3_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMatrix3x3_new(btQuaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMatrix3x3_new(float^ xx, float^ xy, float^ xz, float^ yx, float^ yy, float^ yz, float^ zx, float^ zy, float^ zz);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMatrix3x3_new(btMatrix3x3^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_absolute(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_adjoint(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_cofac(void* obj, int r1, int c1, int r2, int c2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_deSerialize(void* obj, btMatrix3x3FloatData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_deSerializeDouble(void* obj, btMatrix3x3DoubleData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_deSerializeFloat(void* obj, btMatrix3x3FloatData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_determinant(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_diagonalize(void* obj, btMatrix3x3^ rot, float tolerance, int maxIter);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_extractRotation(void* obj, btQuaternion^ q, float tolerance, int maxIter);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_getColumn(void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_getEulerYPR(void* obj, float^ yaw, float^ pitch, float^ roll);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_getEulerZYX(void* obj, float^ yaw, float^ pitch, float^ roll, uint solution_number);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_getIdentity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_getOpenGLSubMatrix(void* obj, float^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_getRotation(void* obj, btQuaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_getRow(void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_inverse(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_operator*=(void* obj, btMatrix3x3^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_operator[](void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_operator[](void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_operator+=(void* obj, btMatrix3x3^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_operator=(void* obj, btMatrix3x3^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_operator-=(void* obj, btMatrix3x3^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_scaled(void* obj, btVector3^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_serialize(void* obj, btMatrix3x3FloatData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_serializeFloat(void* obj, btMatrix3x3FloatData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_setEulerYPR(void* obj, float^ yaw, float^ pitch, float^ roll);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_setEulerZYX(void* obj, float eulerX, float eulerY, float eulerZ);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_setFromOpenGLSubMatrix(void* obj, float^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_setIdentity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_setRotation(void* obj, btQuaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_setValue(void* obj, float^ xx, float^ xy, float^ xz, float^ yx, float^ yy, float^ yz, float^ zx, float^ zy, float^ zz);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_solve33(void* obj, btVector3^ b);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_tdotx(void* obj, btVector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_tdoty(void* obj, btVector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_tdotz(void* obj, btVector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_timesTranspose(void* obj, btMatrix3x3^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_transpose(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrix3x3_transposeTimes(void* obj, btMatrix3x3^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMatrixX_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMatrixX_new(int rows, int cols);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_addElem(void* obj, int row, int col, [unexposed type] val);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_cols(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_copyLowerToUpperTriangle(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_getBufferPointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_getBufferPointerWritable(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_mulElem(void* obj, int row, int col, [unexposed type] val);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_multiply2_p8r(void* obj, int^ B, int^ C, int numRows, int numRowsOther, int row, int col);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_multiplyAdd2_p8r(void* obj, int^ B, int^ C, int numRows, int numRowsOther, int row, int col);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_negative(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_operator()(void* obj, int row, int col);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_operator*(void* obj, btMatrixX^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_printMatrix(void* obj, char^ msg);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_resize(void* obj, int rows, int cols);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_rowComputeNonZeroElements(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_rows(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_setElem(void* obj, int row, int col, [unexposed type] val);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_setIdentity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_setSubMatrix(void* obj, int rowstart, int colstart, int rowend, int colend, [unexposed type] value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_setSubMatrix(void* obj, int rowstart, int colstart, int rowend, int colend, btMatrixX^ block);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_setSubMatrix(void* obj, int rowstart, int colstart, int rowend, int colend, btVectorX^ block);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_setZero(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMatrixX_transpose(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMinkowskiPenetrationDepthSolver_calcPenDepth(void* obj, int^ simplexSolver, btConvexShape^ convexA, btConvexShape^ convexB, btTransform^ transA, btTransform^ transB, btVector3^ v, btVector3^ pa, btVector3^ pb, btIDebugDraw^ debugDraw);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMinkowskiPenetrationDepthSolver_getPenetrationDirections(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMLCPSolver_new(int^ solver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMLCPSolver_createMLCP(void* obj, int^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMLCPSolver_createMLCPFast(void* obj, int^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMLCPSolver_getNumFallbacks(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMLCPSolver_getSolverType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMLCPSolver_setMLCPSolver(void* obj, int^ solver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMLCPSolver_setNumFallbacks(void* obj, int num);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMLCPSolver_solveGroupCacheFriendlyIterations(void* obj, int^ bodies, int numBodies, int^ manifoldPtr, int numManifolds, int^ constraints, int numConstraints, int^ infoGlobal, int^ debugDrawer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMLCPSolver_solveGroupCacheFriendlySetup(void* obj, int^ bodies, int numBodies, int^ manifoldPtr, int numManifolds, int^ constraints, int numConstraints, int^ infoGlobal, int^ debugDrawer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMLCPSolver_solveMLCP(void* obj, int^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMLCPSolverInterface_solveMLCP(void* obj, int^ A, int^ b, int^ x, int^ lo, int^ hi, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMotionState_getWorldTransform(void* obj, btTransform^ worldTrans);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMotionState_setWorldTransform(void* obj, btTransform^ worldTrans);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMprCollisionDescription_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMultiBodyDynamicsWorld_new(int^ dispatcher, int^ pairCache, btMultiBodyConstraintSolver^ constraintSolver, int^ collisionConfiguration);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_addMultiBody(void* obj, btMultiBody^ body, int group, int mask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_addMultiBodyConstraint(void* obj, btMultiBodyConstraint^ constraint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_applyGravity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_calculateSimulationIslands(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_clearForces(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_clearMultiBodyConstraintForces(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_clearMultiBodyForces(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_debugDrawMultiBodyConstraint(void* obj, btMultiBodyConstraint^ constraint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_debugDrawWorld(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_forwardKinematics(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_getMultiBody(void* obj, int mbIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_getMultiBody(void* obj, int mbIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_getMultiBodyConstraint(void* obj, int constraintIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_getMultiBodyConstraint(void* obj, int constraintIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_getNumMultibodies(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_getNumMultiBodyConstraints(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_integrateTransforms(void* obj, int timeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_removeMultiBody(void* obj, btMultiBody^ body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_removeMultiBodyConstraint(void* obj, btMultiBodyConstraint^ constraint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_serialize(void* obj, int^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_serializeMultiBodies(void* obj, int^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_solveConstraints(void* obj, int^ solverInfo);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyDynamicsWorld_updateActivationState(void* obj, int timeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMultiBodyFixedConstraint_new(btMultiBody^ body, int link, int^ bodyB, int^ pivotInA, int^ pivotInB, int^ frameInA, int^ frameInB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMultiBodyFixedConstraint_new(btMultiBody^ bodyA, int linkA, btMultiBody^ bodyB, int linkB, int^ pivotInA, int^ pivotInB, int^ frameInA, int^ frameInB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyFixedConstraint_createConstraintRows(void* obj, int^ constraintRows, btMultiBodyJacobianData^ data, int^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyFixedConstraint_debugDraw(void* obj, btIDebugDraw^ drawer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyFixedConstraint_finalizeMultiDof(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyFixedConstraint_getFrameInA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyFixedConstraint_getFrameInB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyFixedConstraint_getIslandIdA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyFixedConstraint_getIslandIdB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyFixedConstraint_getPivotInA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyFixedConstraint_getPivotInB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyFixedConstraint_setFrameInA(void* obj, int^ frameInA);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyFixedConstraint_setFrameInB(void* obj, int^ frameInB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyFixedConstraint_setPivotInA(void* obj, int^ pivotInA);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyFixedConstraint_setPivotInB(void* obj, int^ pivotInB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMultiBodyGearConstraint_new(btMultiBody^ bodyA, int linkA, btMultiBody^ bodyB, int linkB, int^ pivotInA, int^ pivotInB, int^ frameInA, int^ frameInB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_createConstraintRows(void* obj, int^ constraintRows, btMultiBodyJacobianData^ data, int^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_debugDraw(void* obj, btIDebugDraw^ drawer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_finalizeMultiDof(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_getFrameInA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_getFrameInB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_getIslandIdA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_getIslandIdB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_getPivotInA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_getPivotInB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_setErp(void* obj, int erp);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_setFrameInA(void* obj, int^ frameInA);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_setFrameInB(void* obj, int^ frameInB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_setGearAuxLink(void* obj, int gearAuxLink);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_setGearRatio(void* obj, int gearRatio);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_setPivotInA(void* obj, int^ pivotInA);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_setPivotInB(void* obj, int^ pivotInB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyGearConstraint_setRelativePositionTarget(void* obj, int relPosTarget);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMultiBodyJointLimitConstraint_new(btMultiBody^ body, int link, int lower, int upper);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointLimitConstraint_createConstraintRows(void* obj, int^ constraintRows, btMultiBodyJacobianData^ data, int^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointLimitConstraint_debugDraw(void* obj, btIDebugDraw^ drawer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointLimitConstraint_finalizeMultiDof(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointLimitConstraint_getIslandIdA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointLimitConstraint_getIslandIdB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMultiBodyJointMotor_new(btMultiBody^ body, int link, int desiredVelocity, int maxMotorImpulse);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMultiBodyJointMotor_new(btMultiBody^ body, int link, int linkDoF, int desiredVelocity, int maxMotorImpulse);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointMotor_createConstraintRows(void* obj, int^ constraintRows, btMultiBodyJacobianData^ data, int^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointMotor_debugDraw(void* obj, btIDebugDraw^ drawer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointMotor_finalizeMultiDof(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointMotor_getErp(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointMotor_getIslandIdA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointMotor_getIslandIdB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointMotor_setErp(void* obj, int erp);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointMotor_setPositionTarget(void* obj, int posTarget, int kp);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointMotor_setRhsClamp(void* obj, int rhsClamp);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyJointMotor_setVelocityTarget(void* obj, int velTarget, int kd);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMultibodyLink_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultibodyLink_BT_DECLARE_ALIGNED_ALLOCATOR(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultibodyLink_getAxisBottom(void* obj, int dof);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultibodyLink_getAxisTop(void* obj, int dof);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultibodyLink_setAxisBottom(void* obj, int dof, int^ axis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultibodyLink_setAxisBottom(void* obj, int dof, int^ x, int^ y, int^ z);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultibodyLink_setAxisTop(void* obj, int dof, int^ axis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultibodyLink_setAxisTop(void* obj, int dof, int^ x, int^ y, int^ z);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultibodyLink_updateCacheMultiDof(void* obj, int^ pq);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMultiBodyLinkCollider_new(int^ multiBody, int link);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyLinkCollider_calculateSerializeBufferSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyLinkCollider_checkCollideWithOverride(void* obj, int^ co);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyLinkCollider_serialize(void* obj, void^ dataBuffer, btSerializer^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyLinkCollider_upcast(void* obj, int^ colObj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodyLinkCollider_upcast(void* obj, int^ colObj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMultiBodySliderConstraint_new(btMultiBody^ body, int link, int^ bodyB, int^ pivotInA, int^ pivotInB, int^ frameInA, int^ frameInB, int^ jointAxis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btMultiBodySliderConstraint_new(btMultiBody^ bodyA, int linkA, btMultiBody^ bodyB, int linkB, int^ pivotInA, int^ pivotInB, int^ frameInA, int^ frameInB, int^ jointAxis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_createConstraintRows(void* obj, int^ constraintRows, btMultiBodyJacobianData^ data, int^ infoGlobal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_debugDraw(void* obj, btIDebugDraw^ drawer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_finalizeMultiDof(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_getFrameInA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_getFrameInB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_getIslandIdA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_getIslandIdB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_getJointAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_getPivotInA(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_getPivotInB(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_setFrameInA(void* obj, int^ frameInA);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_setFrameInB(void* obj, int^ frameInB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_setJointAxis(void* obj, int^ jointAxis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_setPivotInA(void* obj, int^ pivotInA);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btMultiBodySliderConstraint_setPivotInB(void* obj, int^ pivotInB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNodeOverlapCallback_processNode(void* obj, int subPart, int triangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_addOverlappingPair(void* obj, btBroadphaseProxy^ , btBroadphaseProxy^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_cleanOverlappingPair(void* obj, btBroadphasePair^ , btDispatcher^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_cleanProxyFromPairs(void* obj, btBroadphaseProxy^ , btDispatcher^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_findPair(void* obj, btBroadphaseProxy^ , btBroadphaseProxy^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_getNumOverlappingPairs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_getOverlappingPairArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_hasDeferredRemoval(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_processAllOverlappingPairs(void* obj, btOverlapCallback^ , btDispatcher^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_removeOverlappingPair(void* obj, btBroadphaseProxy^ , btBroadphaseProxy^ , btDispatcher^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_removeOverlappingPairsContainingProxy(void* obj, btBroadphaseProxy^ , btDispatcher^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_setInternalGhostPairCallback(void* obj, btOverlappingPairCallback^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_setOverlapFilterCallback(void* obj, btOverlapFilterCallback^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btNullPairCache_sortOverlappingPairs(void* obj, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlapCallback_processOverlap(void* obj, btBroadphasePair^ pair);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlapFilterCallback_needBroadphaseCollision(void* obj, btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCache_cleanOverlappingPair(void* obj, btBroadphasePair^ pair, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCache_cleanProxyFromPairs(void* obj, btBroadphaseProxy^ proxy, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCache_findPair(void* obj, btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCache_getNumOverlappingPairs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCache_getOverlappingPairArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCache_hasDeferredRemoval(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCache_processAllOverlappingPairs(void* obj, btOverlapCallback^ , btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCache_setInternalGhostPairCallback(void* obj, btOverlappingPairCallback^ ghostPairCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCache_setOverlapFilterCallback(void* obj, btOverlapFilterCallback^ callback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCache_sortOverlappingPairs(void* obj, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btOverlappingPairCallback_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCallback_addOverlappingPair(void* obj, int^ proxy0, int^ proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCallback_removeOverlappingPair(void* obj, int^ proxy0, int^ proxy1, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btOverlappingPairCallback_removeOverlappingPairsContainingProxy(void* obj, int^ proxy0, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btPairCachingGhostObject_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPairCachingGhostObject_addOverlappingObjectInternal(void* obj, btBroadphaseProxy^ otherProxy, btBroadphaseProxy^ thisProxy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPairCachingGhostObject_getOverlappingPairCache(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPairCachingGhostObject_removeOverlappingObjectInternal(void* obj, btBroadphaseProxy^ otherProxy, btDispatcher^ dispatcher, btBroadphaseProxy^ thisProxy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btPairSet_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPairSet_push_pair(void* obj, int index1, int index2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPairSet_push_pair_inv(void* obj, int index1, int index2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btPlane_new(btVector3^ n, float d);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btPlane_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btPointCollector_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPointCollector_addContactPoint(void* obj, int^ normalOnBInWorld, int^ pointInWorld, int depth);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPointCollector_setShapeIdentifiersA(void* obj, int partId0, int index0);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPointCollector_setShapeIdentifiersB(void* obj, int partId1, int index1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btPolarDecomposition_new(float tolerance, uint maxIterations);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPolarDecomposition_decompose(void* obj, btMatrix3x3^ a, btMatrix3x3^ u, btMatrix3x3^ h);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPolarDecomposition_maxIterations(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPolyhedralContactClipping_clipFace(void* obj, int^ pVtxIn, int^ ppVtxOut, int^ planeNormalWS, int planeEqWS);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPolyhedralContactClipping_clipFaceAgainstHull(void* obj, int^ separatingNormal, btConvexPolyhedron^ hullA, int^ transA, int^ worldVertsB1, int^ worldVertsB2, int minDist, int maxDist, Result^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPolyhedralContactClipping_clipHullAgainstHull(void* obj, int^ separatingNormal1, btConvexPolyhedron^ hullA, btConvexPolyhedron^ hullB, int^ transA, int^ transB, int minDist, int maxDist, int^ worldVertsB1, int^ worldVertsB2, Result^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPolyhedralContactClipping_findSeparatingAxis(void* obj, btConvexPolyhedron^ hullA, btConvexPolyhedron^ hullB, int^ transA, int^ transB, int^ sep, Result^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btPolyhedralConvexAabbCachingShape_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPolyhedralConvexAabbCachingShape_getAabb(void* obj, int^ t, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPolyhedralConvexAabbCachingShape_getCachedLocalAabb(void* obj, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPolyhedralConvexAabbCachingShape_getNonvirtualAabb(void* obj, int^ trans, int^ aabbMin, int^ aabbMax, int margin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPolyhedralConvexAabbCachingShape_recalcLocalAabb(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPolyhedralConvexAabbCachingShape_setCachedLocalAabb(void* obj, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPolyhedralConvexAabbCachingShape_setLocalScaling(void* obj, int^ scaling);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btPoolAllocator_new(int elemSize, int maxElements);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPoolAllocator_allocate(void* obj, int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPoolAllocator_freeMemory(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPoolAllocator_getElementSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPoolAllocator_getFreeCount(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPoolAllocator_getMaxCount(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPoolAllocator_getPoolAddress(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPoolAllocator_getPoolAddress(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPoolAllocator_getUsedCount(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPoolAllocator_validPtr(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPrimitiveManagerBase_get_primitive_box(void* obj, int prim_index, int^ primbox);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPrimitiveManagerBase_get_primitive_count(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPrimitiveManagerBase_get_primitive_triangle(void* obj, int prim_index, btPrimitiveTriangle^ triangle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPrimitiveManagerBase_is_trimesh(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btPrimitiveTriangle_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btPrimitiveTriangle_find_triangle_collision_clip_method(void* obj, btPrimitiveTriangle^ other, int^ contacts);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btQuadWord_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btQuadWord_new(float^ _x, float^ _y, float^ _z);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btQuadWord_new(float^ _x, float^ _y, float^ _z, float^ _w);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_getX(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_getY(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_getZ(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_operator!=(void* obj, btQuadWord^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_operator==(void* obj, btQuadWord^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_setMax(void* obj, btQuadWord^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_setMin(void* obj, btQuadWord^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_setValue(void* obj, float^ _x, float^ _y, float^ _z);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_setValue(void* obj, float^ _x, float^ _y, float^ _z, float^ _w);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_setW(void* obj, float _w);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_setX(void* obj, float _x);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_setY(void* obj, float _y);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_setZ(void* obj, float _z);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_w(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_x(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_y(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuadWord_z(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btQuantizedBvhTree_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuantizedBvhTree__build_sub_tree(void* obj, GIM_BVH_DATA_ARRAY^ primitive_boxes, int startIndex, int endIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuantizedBvhTree__calc_splitting_axis(void* obj, GIM_BVH_DATA_ARRAY^ primitive_boxes, int startIndex, int endIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuantizedBvhTree__sort_and_calc_splitting_index(void* obj, GIM_BVH_DATA_ARRAY^ primitive_boxes, int startIndex, int endIndex, int splitAxis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuantizedBvhTree_build_tree(void* obj, GIM_BVH_DATA_ARRAY^ primitive_boxes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuantizedBvhTree_calc_quantization(void* obj, GIM_BVH_DATA_ARRAY^ primitive_boxes, int boundMargin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btQuaternion_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btQuaternion_new(float^ _x, float^ _y, float^ _z, float^ _w);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btQuaternion_new(btVector3^ _axis, float^ _angle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btQuaternion_new(float^ yaw, float^ pitch, float^ roll);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_angle(void* obj, btQuaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_angleShortestPath(void* obj, btQuaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_deSerialize(void* obj, btQuaternionFloatData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_deSerializeDouble(void* obj, btQuaternionDoubleData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_deSerializeFloat(void* obj, btQuaternionFloatData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_dot(void* obj, btQuaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_farthest(void* obj, btQuaternion^ qd);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_getAngle(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_getAngleShortestPath(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_getAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_getEulerZYX(void* obj, float^ yawZ, float^ pitchY, float^ rollX);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_getIdentity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_getW(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_inverse(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_length(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_length2(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_nearest(void* obj, btQuaternion^ qd);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_normalize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_normalized(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_operator-(void* obj, btQuaternion^ q2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_operator-(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_operator*(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_operator*=(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_operator*=(void* obj, btQuaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_operator/(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_operator/=(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_operator+(void* obj, btQuaternion^ q2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_operator+=(void* obj, btQuaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_operator-=(void* obj, btQuaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_safeNormalize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_serialize(void* obj, btQuaternionFloatData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_serializeDouble(void* obj, btQuaternionDoubleData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_serializeFloat(void* obj, btQuaternionFloatData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_setEuler(void* obj, float^ yaw, float^ pitch, float^ roll);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_setEulerZYX(void* obj, float^ yawZ, float^ pitchY, float^ rollX);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_setRotation(void* obj, btVector3^ axis, float^ _angle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btQuaternion_slerp(void* obj, btQuaternion^ q, float^ t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btRaycastVehicle_new(btVehicleTuning^ tuning, btRigidBody^ chassis, btVehicleRaycaster^ raycaster);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_addWheel(void* obj, int^ connectionPointCS0, int^ wheelDirectionCS0, int^ wheelAxleCS, int suspensionRestLength, int wheelRadius, btVehicleTuning^ tuning, bool isFrontWheel);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_applyEngineForce(void* obj, int force, int wheel);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_debugDraw(void* obj, int^ debugDrawer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_defaultInit(void* obj, btVehicleTuning^ tuning);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getChassisWorldTransform(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getCurrentSpeedKmHour(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getForwardAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getForwardVector(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getNumWheels(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getRightAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getRigidBody(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getRigidBody(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getSteeringValue(void* obj, int wheel);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getUpAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getUserConstraintId(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getUserConstraintType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getWheelInfo(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getWheelInfo(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_getWheelTransformWS(void* obj, int wheelIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_rayCast(void* obj, btWheelInfo^ wheel);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_resetSuspension(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_setBrake(void* obj, int brake, int wheelIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_setCoordinateSystem(void* obj, int rightIndex, int upIndex, int forwardIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_setPitchControl(void* obj, int pitch);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_setSteeringValue(void* obj, int steering, int wheel);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_setUserConstraintId(void* obj, int uid);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_setUserConstraintType(void* obj, int userConstraintType);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_updateAction(void* obj, int^ collisionWorld, int step);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_updateFriction(void* obj, int timeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_updateSuspension(void* obj, int deltaTime);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_updateVehicle(void* obj, int step);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_updateWheelTransform(void* obj, int wheelIndex, bool interpolatedTransform);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRaycastVehicle_updateWheelTransformsWS(void* obj, btWheelInfo^ wheel, bool interpolatedTransform);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btRigidBody_new(btRigidBodyConstructionInfo^ constructionInfo);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btRigidBody_new(int mass, btMotionState^ motionState, btCollisionShape^ collisionShape, int^ localInertia);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_addConstraintRef(void* obj, btTypedConstraint^ c);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_applyCentralForce(void* obj, int^ force);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_applyCentralImpulse(void* obj, int^ impulse);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_applyDamping(void* obj, int timeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_applyForce(void* obj, int^ force, int^ rel_pos);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_applyGravity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_applyImpulse(void* obj, int^ impulse, int^ rel_pos);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_applyTorque(void* obj, int^ torque);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_applyTorqueImpulse(void* obj, int^ torque);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_ATTRIBUTE_ALIGNED16(void* obj, int m_deltaLinearVelocity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_calculateSerializeBufferSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_clearForces(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_computeGyroscopicForceExplicit(void* obj, int maxGyroscopicForce);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_computeGyroscopicImpulseImplicit_Body(void* obj, int step);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_computeGyroscopicImpulseImplicit_World(void* obj, int dt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getAabb(void* obj, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getAngularDamping(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getAngularFactor(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getAngularSleepingThreshold(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getAngularVelocity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getCenterOfMassPosition(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getCenterOfMassTransform(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getConstraintRef(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getFlags(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getGravity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getInvInertiaDiagLocal(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getInvInertiaTensorWorld(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getInvMass(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getLinearDamping(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getLinearFactor(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getLinearSleepingThreshold(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getLinearVelocity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getLocalInertia(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getNumConstraintRefs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getOrientation(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getTotalForce(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getTotalTorque(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_getVelocityInLocalPoint(void* obj, int^ rel_pos);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_integrateVelocities(void* obj, int step);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_isInWorld(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_predictIntegratedTransform(void* obj, int step, int^ predictedTransform);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_proceedToTransform(void* obj, int^ newTrans);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_removeConstraintRef(void* obj, btTypedConstraint^ c);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_saveKinematicState(void* obj, int step);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_serialize(void* obj, void^ dataBuffer, btSerializer^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_serializeSingleObject(void* obj, btSerializer^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_setAngularFactor(void* obj, int^ angFac);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_setAngularFactor(void* obj, int angFac);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_setAngularVelocity(void* obj, int^ ang_vel);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_setCenterOfMassTransform(void* obj, int^ xform);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_setDamping(void* obj, int lin_damping, int ang_damping);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_setFlags(void* obj, int flags);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_setGravity(void* obj, int^ acceleration);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_setInvInertiaDiagLocal(void* obj, int^ diagInvInertia);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_setLinearFactor(void* obj, int^ linearFactor);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_setLinearVelocity(void* obj, int^ lin_vel);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_setSleepingThresholds(void* obj, int linear, int angular);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_setupRigidBody(void* obj, btRigidBodyConstructionInfo^ constructionInfo);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_translate(void* obj, int^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_upcast(void* obj, int^ colObj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_upcast(void* obj, int^ colObj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRigidBody_updateInertiaTensor(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btRigidBodyConstructionInfo_new(int mass, btMotionState^ motionState, btCollisionShape^ collisionShape, int^ localInertia);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btRotationalLimitMotor_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btRotationalLimitMotor_new(btRotationalLimitMotor^ limot);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRotationalLimitMotor_isLimited(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRotationalLimitMotor_needApplyTorques(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRotationalLimitMotor_solveAngularLimits(void* obj, int timeStep, int^ axis, int jacDiagABInv, btRigidBody^ body0, btRigidBody^ body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRotationalLimitMotor_testLimitValue(void* obj, int test_value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btRotationalLimitMotor2_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btRotationalLimitMotor2_new(btRotationalLimitMotor2^ limot);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRotationalLimitMotor2_isLimited(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btRotationalLimitMotor2_testLimitValue(void* obj, int test_value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_allocate(void* obj, int size, int numElements);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_finalizeChunk(void* obj, btChunk^ chunk, char^ structType, int chunkCode, void^ oldPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_findNameForPointer(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_findPointer(void* obj, void^ oldPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_finishSerialization(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_getBufferPointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_getChunk(void* obj, int chunkIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_getCurrentBufferSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_getNumChunks(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_getSerializationFlags(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_getUniquePointer(void* obj, void^ oldPtr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_registerNameForPointer(void* obj, void^ ptr, char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_serializeName(void* obj, char^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_setSerializationFlags(void* obj, int flags);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSerializer_startSerialization(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSimpleBroadphase_new(int maxProxies, btOverlappingPairCache^ overlappingPairCache);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_aabbOverlap(void* obj, btSimpleBroadphaseProxy^ proxy0, btSimpleBroadphaseProxy^ proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_aabbTest(void* obj, int^ aabbMin, int^ aabbMax, btBroadphaseAabbCallback^ callback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_allocHandle(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_calculateOverlappingPairs(void* obj, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_createProxy(void* obj, int^ aabbMin, int^ aabbMax, int shapeType, void^ userPtr, int collisionFilterGroup, int collisionFilterMask, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_destroyProxy(void* obj, btBroadphaseProxy^ proxy, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_freeHandle(void* obj, btSimpleBroadphaseProxy^ proxy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_getAabb(void* obj, btBroadphaseProxy^ proxy, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_getBroadphaseAabb(void* obj, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_getOverlappingPairCache(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_getOverlappingPairCache(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_getSimpleProxyFromProxy(void* obj, btBroadphaseProxy^ proxy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_getSimpleProxyFromProxy(void* obj, btBroadphaseProxy^ proxy);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_printStats(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_rayTest(void* obj, int^ rayFrom, int^ rayTo, btBroadphaseRayCallback^ rayCallback, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_resetPool(void* obj, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_setAabb(void* obj, btBroadphaseProxy^ proxy, int^ aabbMin, int^ aabbMax, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_testAabbOverlap(void* obj, btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleBroadphase_validate(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSimpleBroadphaseProxy_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSimpleBroadphaseProxy_new(int^ minpt, int^ maxpt, int shapeType, void^ userPtr, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSimpleDynamicsWorld_new(btDispatcher^ dispatcher, int^ pairCache, btConstraintSolver^ constraintSolver, int^ collisionConfiguration);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_addAction(void* obj, btActionInterface^ action);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_addRigidBody(void* obj, int^ body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_addRigidBody(void* obj, int^ body, int group, int mask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_clearForces(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_debugDrawWorld(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_getConstraintSolver(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_getGravity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_getWorldType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_integrateTransforms(void* obj, int timeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_predictUnconstraintMotion(void* obj, int timeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_removeAction(void* obj, btActionInterface^ action);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_removeCollisionObject(void* obj, int^ collisionObject);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_removeRigidBody(void* obj, int^ body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_setConstraintSolver(void* obj, btConstraintSolver^ solver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_setGravity(void* obj, int^ gravity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_stepSimulation(void* obj, int timeStep, int maxSubSteps, int fixedTimeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_synchronizeMotionStates(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimpleDynamicsWorld_updateAabbs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSimplePair_new(int indexA, int indexB);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSimulationIslandManager_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManager_buildAndProcessIslands(void* obj, btDispatcher^ dispatcher, btCollisionWorld^ collisionWorld, IslandCallback^ callback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManager_buildIslands(void* obj, btDispatcher^ dispatcher, btCollisionWorld^ colWorld);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManager_findUnions(void* obj, btDispatcher^ dispatcher, btCollisionWorld^ colWorld);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManager_getSplitIslands(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManager_getUnionFind(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManager_initUnionFind(void* obj, int n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManager_setSplitIslands(void* obj, bool doSplitIslands);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManager_storeIslandActivationState(void* obj, btCollisionWorld^ world);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManager_updateActivationState(void* obj, btCollisionWorld^ colWorld, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSimulationIslandManagerMt_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_addBodiesToIslands(void* obj, int^ collisionWorld);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_addConstraintsToIslands(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_addManifoldsToIslands(void* obj, int^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_allocateIsland(void* obj, int id, int numBodies);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_buildAndProcessIslands(void* obj, int^ dispatcher, int^ collisionWorld, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_buildIslands(void* obj, int^ dispatcher, int^ colWorld);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_getIsland(void* obj, int id);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_getIslandDispatchFunction(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_getMinimumSolverBatchSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_initIslandPools(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_mergeIslands(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_parallelIslandDispatch(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_serialIslandDispatch(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_setIslandDispatchFunction(void* obj, IslandDispatchFunc func);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSimulationIslandManagerMt_setMinimumSolverBatchSize(void* obj, int sz);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftBody_new(btSoftBodyWorldInfo^ worldInfo, int node_count, int^ x, int^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftBody_new(btSoftBodyWorldInfo^ worldInfo);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_addAeroForceToFace(void* obj, int^ windVelocity, int faceIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_addAeroForceToNode(void* obj, int^ windVelocity, int nodeIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_addForce(void* obj, int^ force);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_addForce(void* obj, int^ force, int node);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_addVelocity(void* obj, int^ velocity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_addVelocity(void* obj, int^ velocity, int node);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendAnchor(void* obj, int node, int^ body, bool disableCollisionBetweenLinkedBodies, int influence);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendAnchor(void* obj, int node, int^ body, int^ localPivot, bool disableCollisionBetweenLinkedBodies, int influence);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendAngularJoint(void* obj, Specs^ specs, Cluster^ body0, Body body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendAngularJoint(void* obj, Specs^ specs, Body body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendAngularJoint(void* obj, Specs^ specs, btSoftBody^ body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendFace(void* obj, int model, Material^ mat);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendFace(void* obj, int node0, int node1, int node2, Material^ mat);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendLinearJoint(void* obj, Specs^ specs, Cluster^ body0, Body body1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendLinearJoint(void* obj, Specs^ specs, Body body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendLinearJoint(void* obj, Specs^ specs, btSoftBody^ body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendLink(void* obj, int model, Material^ mat);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendLink(void* obj, int node0, int node1, Material^ mat, bool bcheckexist);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendLink(void* obj, Node^ node0, Node^ node1, Material^ mat, bool bcheckexist);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendMaterial(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendNode(void* obj, int^ x, int m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendNote(void* obj, char^ text, int^ o, int^ c, Node^ n0, Node^ n1, Node^ n2, Node^ n3);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendNote(void* obj, char^ text, int^ o, Node^ feature);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendNote(void* obj, char^ text, int^ o, int^ feature);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendNote(void* obj, char^ text, int^ o, Face^ feature);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendTetra(void* obj, int model, Material^ mat);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_appendTetra(void* obj, int node0, int node1, int node2, int node3, Material^ mat);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_applyClusters(void* obj, bool drift);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_applyForces(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_ATTRIBUTE_ALIGNED16(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_calculateSerializeBufferSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_checkContact(void* obj, int^ colObjWrap, int^ x, int margin, sCti^ cti);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_checkFace(void* obj, int node0, int node1, int node2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_checkLink(void* obj, int node0, int node1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_checkLink(void* obj, Node^ node0, Node^ node1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_cleanupClusters(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_clusterAImpulse(void* obj, Cluster^ cluster, Impulse^ impulse);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_clusterCom(void* obj, Cluster^ cluster);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_clusterCom(void* obj, int cluster);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_clusterCount(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_clusterDAImpulse(void* obj, Cluster^ cluster, int^ impulse);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_clusterDCImpulse(void* obj, Cluster^ cluster, int^ impulse);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_clusterDImpulse(void* obj, Cluster^ cluster, int^ rpos, int^ impulse);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_clusterImpulse(void* obj, Cluster^ cluster, int^ rpos, Impulse^ impulse);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_clusterVAImpulse(void* obj, Cluster^ cluster, int^ impulse);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_clusterVelocity(void* obj, Cluster^ cluster, int^ rpos);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_clusterVImpulse(void* obj, Cluster^ cluster, int^ rpos, int^ impulse);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_cutLink(void* obj, int node0, int node1, int position);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_cutLink(void* obj, Node^ node0, Node^ node1, int position);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_dampClusters(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_defaultCollisionHandler(void* obj, int^ pcoWrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_defaultCollisionHandler(void* obj, btSoftBody^ psb);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_evaluateCom(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_generateBendingConstraints(void* obj, int distance, Material^ mat);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_generateClusters(void* obj, int k, int maxiterations);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_getAabb(void* obj, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_getMass(void* obj, int node);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_getRestLengthScale(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_getSoftBodySolver(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_getSoftBodySolver(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_getSolver(void* obj, _ solver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_getSolver(void* obj, _ solver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_getTotalMass(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_getVolume(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_getWindVelocity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_getWorldInfo(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_indicesToPointers(void* obj, int^ map);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_initDefaults(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_initializeClusters(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_initializeFaceTree(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_integrateMotion(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_pointersToIndices(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_predictMotion(void* obj, int dt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_prepareClusters(void* obj, int iterations);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_PSolve_Anchors(void* obj, btSoftBody^ psb, int kst, int ti);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_PSolve_Links(void* obj, btSoftBody^ psb, int kst, int ti);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_PSolve_RContacts(void* obj, btSoftBody^ psb, int kst, int ti);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_PSolve_SContacts(void* obj, btSoftBody^ psb, int btScalar, int ti);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_randomizeConstraints(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_rayTest(void* obj, int^ rayFrom, int^ rayTo, sRayCast^ results);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_rayTest(void* obj, int^ rayFrom, int^ rayTo, int^ mint, _^ feature, int^ index, bool bcountonly);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_refine(void* obj, ImplicitFn^ ifn, int accurary, bool cut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_releaseCluster(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_releaseClusters(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_resetLinkRestLengths(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_rotate(void* obj, int^ rot);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_scale(void* obj, int^ scl);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_serialize(void* obj, void^ dataBuffer, btSerializer^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_setCollisionShape(void* obj, int^ collisionShape);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_setMass(void* obj, int node, int mass);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_setPose(void* obj, bool bvolume, bool bframe);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_setRestLengthScale(void* obj, int restLength);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_setSoftBodySolver(void* obj, btSoftBodySolver^ softBodySolver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_setSolver(void* obj, _ preset);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_setTotalDensity(void* obj, int density);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_setTotalMass(void* obj, int mass, bool fromfaces);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_setVelocity(void* obj, int^ velocity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_setVolumeDensity(void* obj, int density);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_setVolumeMass(void* obj, int mass);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_setWindVelocity(void* obj, int^ velocity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_solveClusters(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_solveClusters(void* obj, int sor);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_solveCommonConstraints(void* obj, btSoftBody^ bodies, int count, int iterations);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_solveConstraints(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_staticSolve(void* obj, int iterations);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_transform(void* obj, int^ trs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_translate(void* obj, int^ trs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_upcast(void* obj, int^ colObj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_upcast(void* obj, int^ colObj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_updateArea(void* obj, bool averageArea);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_updateBounds(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_updateClusters(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_updateConstants(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_updateLinkConstants(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_updateNormals(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_updatePose(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBody_VSolve_Links(void* obj, btSoftBody^ psb, int kst);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftBodyCollisionShape_new(btSoftBody^ backptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyCollisionShape_calculateLocalInertia(void* obj, int btScalar, int^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyCollisionShape_getAabb(void* obj, int^ t, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyCollisionShape_getLocalScaling(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyCollisionShape_getName(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyCollisionShape_processAllTriangles(void* obj, int^ , int^ , int^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyCollisionShape_setLocalScaling(void* obj, int^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftBodyConcaveCollisionAlgorithm_new(int^ ci, int^ body0Wrap, int^ body1Wrap, bool isSwapped);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyConcaveCollisionAlgorithm_calculateTimeOfImpact(void* obj, int^ body0, int^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyConcaveCollisionAlgorithm_clearCache(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyConcaveCollisionAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyConcaveCollisionAlgorithm_processCollision(void* obj, int^ body0Wrap, int^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_CalculateUV(void* obj, int resx, int resy, int ix, int iy, int id);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_CreateEllipsoid(void* obj, btSoftBodyWorldInfo^ worldInfo, int^ center, int^ radius, int res);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_CreateFromConvexHull(void* obj, btSoftBodyWorldInfo^ worldInfo, int^ vertices, int nvertices, bool randomizeConstraints);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_CreateFromTetGenData(void* obj, btSoftBodyWorldInfo^ worldInfo, char^ ele, char^ face, char^ node, bool bfacelinks, bool btetralinks, bool bfacesfromtetras);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_CreateFromTriMesh(void* obj, btSoftBodyWorldInfo^ worldInfo, int^ vertices, int^ triangles, int ntriangles, bool randomizeConstraints);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_CreatePatch(void* obj, btSoftBodyWorldInfo^ worldInfo, int^ corner00, int^ corner10, int^ corner01, int^ corner11, int resx, int resy, int fixeds, bool gendiags);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_CreatePatchUV(void* obj, btSoftBodyWorldInfo^ worldInfo, int^ corner00, int^ corner10, int^ corner01, int^ corner11, int resx, int resy, int fixeds, bool gendiags, float^ tex_coords);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_CreateRope(void* obj, btSoftBodyWorldInfo^ worldInfo, int^ from, int^ to, int res, int fixeds);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_Draw(void* obj, btSoftBody^ psb, int^ idraw, int drawflags);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_DrawClusterTree(void* obj, btSoftBody^ psb, int^ idraw, int mindepth, int maxdepth);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_DrawFaceTree(void* obj, btSoftBody^ psb, int^ idraw, int mindepth, int maxdepth);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_DrawFrame(void* obj, btSoftBody^ psb, int^ idraw);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_DrawInfos(void* obj, btSoftBody^ psb, int^ idraw, bool masses, bool areas, bool stress);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_DrawNodeTree(void* obj, btSoftBody^ psb, int^ idraw, int mindepth, int maxdepth);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyHelpers_ReoptimizeLinkOrder(void* obj, btSoftBody^ psb);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftBodyRigidBodyCollisionConfiguration_new(int^ constructionInfo);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyRigidBodyCollisionConfiguration_getCollisionAlgorithmCreateFunc(void* obj, int proxyType0, int proxyType1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftBodySolver_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolver_checkInitialized(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolver_copyBackToSoftBodies(void* obj, bool bMove);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolver_getNumberOfPositionIterations(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolver_getNumberOfVelocityIterations(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolver_getSolverType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolver_getTimeScale(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolver_optimize(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolver_predictMotion(void* obj, float solverdt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolver_processCollision(void* obj, btSoftBody^ , btCollisionObjectWrapper^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolver_processCollision(void* obj, btSoftBody^ , btSoftBody^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolver_setNumberOfPositionIterations(void* obj, int iterations);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolver_setNumberOfVelocityIterations(void* obj, int iterations);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolver_solveConstraints(void* obj, float solverdt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolver_updateSoftBodies(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftBodySolverOutput_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodySolverOutput_copySoftBodyToVertexBuffer(void* obj, btSoftBody^ softBody, btVertexBufferDescriptor^ vertexBuffer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftBodyTriangleCallback_new(btDispatcher^ dispatcher, int^ body0Wrap, int^ body1Wrap, bool isSwapped);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyTriangleCallback_clearCache(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyTriangleCallback_processTriangle(void* obj, int^ triangle, int partId, int triangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftBodyTriangleCallback_setTimeStepAndCounters(void* obj, int collisionMarginTriangle, int^ triObjWrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftBodyWorldInfo_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftClusterCollisionShape_new(Cluster^ cluster);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftClusterCollisionShape_batchedUnitVectorGetSupportingVertexWithoutMargin(void* obj, int^ vectors, int^ supportVerticesOut, int numVectors);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftClusterCollisionShape_calculateLocalInertia(void* obj, int mass, int^ inertia);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftClusterCollisionShape_getAabb(void* obj, int^ t, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftClusterCollisionShape_getMargin(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftClusterCollisionShape_getName(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftClusterCollisionShape_getShapeType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftClusterCollisionShape_localGetSupportingVertex(void* obj, int^ vec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftClusterCollisionShape_localGetSupportingVertexWithoutMargin(void* obj, int^ vec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftClusterCollisionShape_setMargin(void* obj, int margin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftMultiBodyDynamicsWorld_new(int^ dispatcher, int^ pairCache, int^ constraintSolver, int^ collisionConfiguration, btSoftBodySolver^ softBodySolver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_addSoftBody(void* obj, int^ body, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_debugDrawWorld(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_getDrawFlags(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_getSoftBodyArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_getSoftBodyArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_getWorldInfo(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_getWorldInfo(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_getWorldType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_internalSingleStepSimulation(void* obj, int timeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_predictUnconstraintMotion(void* obj, int timeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_rayTest(void* obj, int^ rayFromWorld, int^ rayToWorld, int^ resultCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_rayTestSingle(void* obj, int^ rayFromTrans, int^ rayToTrans, int^ collisionObject, int^ collisionShape, int^ colObjWorldTransform, int^ resultCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_removeCollisionObject(void* obj, int^ collisionObject);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_removeSoftBody(void* obj, int^ body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_serialize(void* obj, int^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_serializeSoftBodies(void* obj, int^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_setDrawFlags(void* obj, int f);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftMultiBodyDynamicsWorld_solveSoftBodiesConstraints(void* obj, int timeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftRigidCollisionAlgorithm_new(btPersistentManifold^ mf, int^ ci, int^ col0, int^ col1Wrap, bool isSwapped);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidCollisionAlgorithm_calculateTimeOfImpact(void* obj, int^ body0, int^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidCollisionAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidCollisionAlgorithm_processCollision(void* obj, int^ body0Wrap, int^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftRigidDynamicsWorld_new(btDispatcher^ dispatcher, btBroadphaseInterface^ pairCache, int^ constraintSolver, int^ collisionConfiguration, btSoftBodySolver^ softBodySolver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_addSoftBody(void* obj, btSoftBody^ body, int collisionFilterGroup, int collisionFilterMask);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_debugDrawWorld(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_getDrawFlags(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_getSoftBodyArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_getSoftBodyArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_getWorldInfo(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_getWorldInfo(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_getWorldType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_internalSingleStepSimulation(void* obj, int timeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_predictUnconstraintMotion(void* obj, int timeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_rayTest(void* obj, int^ rayFromWorld, int^ rayToWorld, int^ resultCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_rayTestSingle(void* obj, int^ rayFromTrans, int^ rayToTrans, int^ collisionObject, int^ collisionShape, int^ colObjWorldTransform, int^ resultCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_removeCollisionObject(void* obj, int^ collisionObject);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_removeSoftBody(void* obj, btSoftBody^ body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_serialize(void* obj, btSerializer^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_serializeSoftBodies(void* obj, btSerializer^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_setDrawFlags(void* obj, int f);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftRigidDynamicsWorld_solveSoftBodiesConstraints(void* obj, int timeStep);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftSoftCollisionAlgorithm_new(int^ ci);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSoftSoftCollisionAlgorithm_new(btPersistentManifold^ mf, int^ ci, int^ body0Wrap, int^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftSoftCollisionAlgorithm_calculateTimeOfImpact(void* obj, int^ body0, int^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftSoftCollisionAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSoftSoftCollisionAlgorithm_processCollision(void* obj, int^ body0Wrap, int^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSolve2LinearConstraint_new(int tau, int damping);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSolve2LinearConstraint_resolveBilateralPairConstraint(void* obj, btRigidBody^ body0, btRigidBody^ body1, int^ world2A, int^ world2B, int^ invInertiaADiag, int invMassA, int^ linvelA, int^ angvelA, int^ rel_posA1, int^ invInertiaBDiag, int invMassB, int^ linvelB, int^ angvelB, int^ rel_posA2, int depthA, int^ normalA, int^ rel_posB1, int^ rel_posB2, int depthB, int^ normalB, int^ imp0, int^ imp1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSolve2LinearConstraint_resolveUnilateralPairConstraint(void* obj, btRigidBody^ body0, btRigidBody^ body1, int^ world2A, int^ world2B, int^ invInertiaADiag, int invMassA, int^ linvelA, int^ angvelA, int^ rel_posA1, int^ invInertiaBDiag, int invMassB, int^ linvelB, int^ angvelB, int^ rel_posA2, int depthA, int^ normalA, int^ rel_posB1, int^ rel_posB2, int depthB, int^ normalB, int^ imp0, int^ imp1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSolveProjectedGaussSeidel_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSolveProjectedGaussSeidel_solveMLCP(void* obj, int^ A, int^ b, int^ x, int^ lo, int^ hi, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSortedOverlappingPairCache_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_addOverlappingPair(void* obj, btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_cleanOverlappingPair(void* obj, btBroadphasePair^ pair, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_cleanProxyFromPairs(void* obj, btBroadphaseProxy^ proxy, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_findPair(void* obj, btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_getNumOverlappingPairs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_getOverlapFilterCallback(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_getOverlappingPairArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_getOverlappingPairArray(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_getOverlappingPairArrayPtr(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_hasDeferredRemoval(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_needsBroadphaseCollision(void* obj, btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_processAllOverlappingPairs(void* obj, btOverlapCallback^ , btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_removeOverlappingPair(void* obj, btBroadphaseProxy^ proxy0, btBroadphaseProxy^ proxy1, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_removeOverlappingPairsContainingProxy(void* obj, btBroadphaseProxy^ proxy, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_setInternalGhostPairCallback(void* obj, btOverlappingPairCallback^ ghostPairCallback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_setOverlapFilterCallback(void* obj, btOverlapFilterCallback^ callback);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSortedOverlappingPairCache_sortOverlappingPairs(void* obj, btDispatcher^ dispatcher);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSparseSdf_BuildCell(void* obj, Cell^ c);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSparseSdf_Decompose(void* obj, int x);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSparseSdf_DistanceToShape(void* obj, int^ x, int^ shape);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSparseSdf_Evaluate(void* obj, int^ x, int^ shape, int^ normal, int margin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSparseSdf_GarbageCollect(void* obj, int lifetime);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSparseSdf_Hash(void* obj, int x, int y, int z, int^ shape);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSparseSdf_Initialize(void* obj, int hashsize, int clampCells);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSparseSdf_Lerp(void* obj, int a, int b, int t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSparseSdf_RemoveReferences(void* obj, int^ pcs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSparseSdf_Reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSpatialForceVector_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSpatialForceVector_new(btVector3^ angular, btVector3^ linear);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSpatialForceVector_new(float^ ax, float^ ay, float^ az, float^ lx, float^ ly, float^ lz);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_addAngular(void* obj, btVector3^ angular);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_addLinear(void* obj, btVector3^ linear);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_addValue(void* obj, float^ ax, float^ ay, float^ az, float^ lx, float^ ly, float^ lz);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_addVector(void* obj, btVector3^ angular, btVector3^ linear);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_getAngular(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_getLinear(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_operator-(void* obj, btSpatialForceVector^ vec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_operator-(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_operator*(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_operator+(void* obj, btSpatialForceVector^ vec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_operator+=(void* obj, btSpatialForceVector^ vec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_operator-=(void* obj, btSpatialForceVector^ vec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_setAngular(void* obj, btVector3^ angular);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_setLinear(void* obj, btVector3^ linear);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_setValue(void* obj, float^ ax, float^ ay, float^ az, float^ lx, float^ ly, float^ lz);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_setVector(void* obj, btVector3^ angular, btVector3^ linear);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialForceVector_setZero(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSpatialMotionVector_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSpatialMotionVector_new(btVector3^ angular, btVector3^ linear);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_addAngular(void* obj, btVector3^ angular);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_addLinear(void* obj, btVector3^ linear);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_addValue(void* obj, float^ ax, float^ ay, float^ az, float^ lx, float^ ly, float^ lz);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_addVector(void* obj, btVector3^ angular, btVector3^ linear);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_dot(void* obj, btSpatialForceVector^ b);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_getAngular(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_getLinear(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_operator-(void* obj, btSpatialMotionVector^ vec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_operator-(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_operator*(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_operator*=(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_operator+(void* obj, btSpatialMotionVector^ vec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_operator+=(void* obj, btSpatialMotionVector^ vec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_operator-=(void* obj, btSpatialMotionVector^ vec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_setAngular(void* obj, btVector3^ angular);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_setLinear(void* obj, btVector3^ linear);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_setValue(void* obj, float^ ax, float^ ay, float^ az, float^ lx, float^ ly, float^ lz);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_setVector(void* obj, btVector3^ angular, btVector3^ linear);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialMotionVector_setZero(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpatialTransformationMatrix_transformInverse(void* obj, btSymmetricSpatialDyad^ inMat, btSymmetricSpatialDyad^ outMat, eOutputOperation outOp);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSphereBoxCollisionAlgorithm_new(btPersistentManifold^ mf, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, bool isSwapped);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSphereBoxCollisionAlgorithm_calculateTimeOfImpact(void* obj, btCollisionObject^ body0, btCollisionObject^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSphereBoxCollisionAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSphereBoxCollisionAlgorithm_getSphereDistance(void* obj, btCollisionObjectWrapper^ boxObjWrap, int^ v3PointOnBox, int^ normal, int^ penetrationDepth, int^ v3SphereCenter, int fRadius, int maxContactDistance);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSphereBoxCollisionAlgorithm_getSpherePenetration(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSphereBoxCollisionAlgorithm_processCollision(void* obj, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSphereSphereCollisionAlgorithm_new(btPersistentManifold^ mf, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ col0Wrap, btCollisionObjectWrapper^ col1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSphereSphereCollisionAlgorithm_new(btCollisionAlgorithmConstructionInfo^ ci);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSphereSphereCollisionAlgorithm_calculateTimeOfImpact(void* obj, btCollisionObject^ body0, btCollisionObject^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSphereSphereCollisionAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSphereSphereCollisionAlgorithm_processCollision(void* obj, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSphereTriangleCollisionAlgorithm_new(btPersistentManifold^ mf, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, bool swapped);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSphereTriangleCollisionAlgorithm_new(btCollisionAlgorithmConstructionInfo^ ci);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSphereTriangleCollisionAlgorithm_calculateTimeOfImpact(void* obj, btCollisionObject^ body0, btCollisionObject^ body1, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSphereTriangleCollisionAlgorithm_getAllContactManifolds(void* obj, int^ manifoldArray);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSphereTriangleCollisionAlgorithm_processCollision(void* obj, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap, int^ dispatchInfo, int^ resultOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSpinMutex_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpinMutex_lock(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpinMutex_tryLock(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSpinMutex_unlock(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btStackAlloc_new(uint size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btStackAlloc_allocate(void* obj, uint size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btStackAlloc_beginBlock(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btStackAlloc_create(void* obj, uint size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btStackAlloc_ctor(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btStackAlloc_destroy(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btStackAlloc_endBlock(void* obj, btBlock^ block);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btStackAlloc_getAvailableMemory(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btStorageResult_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btStorageResult_addContactPoint(void* obj, int^ normalOnBInWorld, int^ pointInWorld, int depth);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSubSimplexClosestResult_isValid(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSubSimplexClosestResult_reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSubSimplexClosestResult_setBarycentricCoordinates(void* obj, int a, int b, int c, int d);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSubsimplexConvexCast_new(btConvexShape^ shapeA, btConvexShape^ shapeB, int^ simplexSolver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSubsimplexConvexCast_calcTimeOfImpact(void* obj, int^ fromA, int^ toA, int^ fromB, int^ toB, CastResult^ result);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSymMatrix_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSymMatrix_new(int n, [unexposed type]^ init);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSymMatrix_index(void* obj, int c, int r);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSymMatrix_operator()(void* obj, int c, int r);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSymMatrix_operator()(void* obj, int c, int r);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSymMatrix_resize(void* obj, int n, [unexposed type]^ init);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSymmetricSpatialDyad_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btSymmetricSpatialDyad_new(btMatrix3x3^ topLeftMat, btMatrix3x3^ topRightMat, btMatrix3x3^ bottomLeftMat);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSymmetricSpatialDyad_addMatrix(void* obj, btMatrix3x3^ topLeftMat, btMatrix3x3^ topRightMat, btMatrix3x3^ bottomLeftMat);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSymmetricSpatialDyad_operator*(void* obj, btSpatialMotionVector^ vec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSymmetricSpatialDyad_operator-=(void* obj, btSymmetricSpatialDyad^ mat);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSymmetricSpatialDyad_setIdentity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btSymmetricSpatialDyad_setMatrix(void* obj, btMatrix3x3^ topLeftMat, btMatrix3x3^ topRightMat, btMatrix3x3^ bottomLeftMat);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTetrahedronShapeEx_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTransform_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTransform_new(btQuaternion^ q, btVector3^ c);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTransform_new(btMatrix3x3^ b, btVector3^ c);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTransform_new(btTransform^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_deSerialize(void* obj, btTransformFloatData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_deSerializeDouble(void* obj, btTransformDoubleData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_deSerializeFloat(void* obj, btTransformFloatData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_getBasis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_getBasis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_getIdentity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_getOpenGLMatrix(void* obj, float^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_getOrigin(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_getOrigin(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_getRotation(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_inverse(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_inverseTimes(void* obj, btTransform^ t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_invXform(void* obj, btVector3^ inVec);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_mult(void* obj, btTransform^ t1, btTransform^ t2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_operator()(void* obj, btVector3^ x);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_operator*(void* obj, btVector3^ x);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_operator*(void* obj, btQuaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_operator*(void* obj, btTransform^ t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_operator*=(void* obj, btTransform^ t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_operator=(void* obj, btTransform^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_serialize(void* obj, btTransformFloatData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_serializeFloat(void* obj, btTransformFloatData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_setBasis(void* obj, btMatrix3x3^ basis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_setFromOpenGLMatrix(void* obj, float^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_setIdentity(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_setOrigin(void* obj, btVector3^ origin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransform_setRotation(void* obj, btQuaternion^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransformUtil_calculateDiffAxisAngle(void* obj, btTransform^ transform0, btTransform^ transform1, btVector3^ axis, float^ angle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransformUtil_calculateDiffAxisAngleQuaternion(void* obj, btQuaternion^ orn0, btQuaternion^ orn1a, btVector3^ axis, float^ angle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransformUtil_calculateVelocity(void* obj, btTransform^ transform0, btTransform^ transform1, float timeStep, btVector3^ linVel, btVector3^ angVel);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransformUtil_calculateVelocityQuaternion(void* obj, btVector3^ pos0, btVector3^ pos1, btQuaternion^ orn0, btQuaternion^ orn1, float timeStep, btVector3^ linVel, btVector3^ angVel);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTransformUtil_integrateTransform(void* obj, btTransform^ curTrans, btVector3^ linvel, btVector3^ angvel, float timeStep, btTransform^ predictedTransform);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTranslationalLimitMotor_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTranslationalLimitMotor_new(btTranslationalLimitMotor^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTranslationalLimitMotor_isLimited(void* obj, int limitIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTranslationalLimitMotor_needApplyForce(void* obj, int limitIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTranslationalLimitMotor_solveLinearAxis(void* obj, int timeStep, int jacDiagABInv, btRigidBody^ body1, int^ pointInA, btRigidBody^ body2, int^ pointInB, int limit_index, int^ axis_normal_on_a, int^ anchorPos);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTranslationalLimitMotor_testLimitValue(void* obj, int limitIndex, int test_value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTranslationalLimitMotor2_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTranslationalLimitMotor2_new(btTranslationalLimitMotor2^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTranslationalLimitMotor2_isLimited(void* obj, int limitIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTranslationalLimitMotor2_testLimitValue(void* obj, int limitIndex, int test_value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleBuffer_clearBuffer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleBuffer_getNumTriangles(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleBuffer_getTriangle(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleBuffer_processTriangle(void* obj, int^ triangle, int partId, int triangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleCallback_processTriangle(void* obj, int^ triangle, int partId, int triangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTriangleConvexcastCallback_new(btConvexShape^ convexShape, int^ convexShapeFrom, int^ convexShapeTo, int^ triangleToWorld, int triangleCollisionMargin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleConvexcastCallback_processTriangle(void* obj, int^ triangle, int partId, int triangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleConvexcastCallback_reportHit(void* obj, int^ hitNormalLocal, int^ hitPointLocal, int hitFraction, int partId, int triangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTriangleInfo_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTriangleInfoMap_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleInfoMap_calculateSerializeBufferSize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleInfoMap_deSerialize(void* obj, btTriangleInfoMapData^ data);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleInfoMap_serialize(void* obj, void^ dataBuffer, int^ serializer);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTriangleMesh_new(bool use32bitIndices, bool use4componentVertices);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleMesh_addIndex(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleMesh_addTriangle(void* obj, int^ vertex0, int^ vertex1, int^ vertex2, bool removeDuplicateVertices);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleMesh_addTriangleIndices(void* obj, int index1, int index2, int index3);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleMesh_findOrAddVertex(void* obj, int^ vertex, bool removeDuplicateVertices);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleMesh_getNumTriangles(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleMesh_getUse32bitIndices(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleMesh_getUse4componentVertices(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleMesh_preallocateIndices(void* obj, int numindices);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleMesh_preallocateVertices(void* obj, int numverts);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTriangleRaycastCallback_new(int^ from, int^ to, uint flags);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleRaycastCallback_processTriangle(void* obj, int^ triangle, int partId, int triangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleRaycastCallback_reportHit(void* obj, int^ hitNormalLocal, int hitFraction, int partId, int triangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTriangleShapeEx_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTriangleShapeEx_new(int^ p0, int^ p1, int^ p2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTriangleShapeEx_new(btTriangleShapeEx^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleShapeEx_applyTransform(void* obj, int^ t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriangleShapeEx_getAabb(void* obj, int^ t, int^ aabbMin, int^ aabbMax);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTriIndex_new(int partId, int triangleIndex, btCollisionShape^ shape);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriIndex_getPartId(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriIndex_getTriangleIndex(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTriIndex_getUid(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btTypedObject_new(int objectType);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btTypedObject_getObjectType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btUnionFind_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btUnionFind_find(void* obj, int p, int q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btUnionFind_find(void* obj, int x);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btUnionFind_Free(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btUnionFind_reset(void* obj, int N);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btUnionFind_sortIslands(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btUnionFind_unite(void* obj, int p, int q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btUsageBitfield_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btUsageBitfield_reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btVector3_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btVector3_new(float^ _x, float^ _y, float^ _z);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_absolute(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_angle(void* obj, btVector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_closestAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_cross(void* obj, btVector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_deSerialize(void* obj, btVector3FloatData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_deSerializeDouble(void* obj, btVector3DoubleData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_deSerializeFloat(void* obj, btVector3FloatData^ dataIn);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_distance(void* obj, btVector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_distance2(void* obj, btVector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_dot(void* obj, btVector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_dot3(void* obj, btVector3^ v0, btVector3^ v1, btVector3^ v2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_furthestAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_fuzzyZero(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_getSkewSymmetricMatrix(void* obj, btVector3^ v0, btVector3^ v1, btVector3^ v2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_getX(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_getY(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_getZ(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_isZero(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_length(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_length2(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_lerp(void* obj, btVector3^ v, float^ t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_maxAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_maxDot(void* obj, btVector3^ array, long array_count, float^ dotOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_minAxis(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_minDot(void* obj, btVector3^ array, long array_count, float^ dotOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_norm(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_normalize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_normalized(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator delete(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator delete(void* obj, void^ , void^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator delete[](void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator delete[](void* obj, void^ , void^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator new(void* obj, int sizeInBytes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator new(void* obj, int size_t, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator new[](void* obj, int sizeInBytes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator new[](void* obj, int size_t, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator!=(void* obj, btVector3^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator*=(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator*=(void* obj, btVector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator/=(void* obj, float^ s);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator+=(void* obj, btVector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator-=(void* obj, btVector3^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_operator==(void* obj, btVector3^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_rotate(void* obj, btVector3^ wAxis, float angle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_safeNorm(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_safeNormalize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_serialize(void* obj, btVector3FloatData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_serializeDouble(void* obj, btVector3DoubleData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_serializeFloat(void* obj, btVector3FloatData^ dataOut);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_setInterpolate3(void* obj, btVector3^ v0, btVector3^ v1, float rt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_setMax(void* obj, btVector3^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_setMin(void* obj, btVector3^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_setValue(void* obj, float^ _x, float^ _y, float^ _z);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_setW(void* obj, float _w);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_setX(void* obj, float _x);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_setY(void* obj, float _y);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_setZ(void* obj, float _z);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_setZero(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_triple(void* obj, btVector3^ v1, btVector3^ v2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_w(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_x(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_y(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector3_z(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btVector4_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btVector4_new(float^ _x, float^ _y, float^ _z, float^ _w);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector4_absolute4(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector4_closestAxis4(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector4_getW(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector4_maxAxis4(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector4_minAxis4(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVector4_setValue(void* obj, float^ _x, float^ _y, float^ _z, float^ _w);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btVectorX_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btVectorX_new(int numRows);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVectorX_cols(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVectorX_getBufferPointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVectorX_getBufferPointerWritable(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVectorX_nrm2(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVectorX_operator[](void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVectorX_operator[](void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVectorX_resize(void* obj, int rows);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVectorX_rows(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVectorX_setZero(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVectorX_size(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVehicleRaycaster_castRay(void* obj, int^ from, int^ to, btVehicleRaycasterResult^ result);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btVehicleRaycasterResult_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btVehicleTuning_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btVertexBufferDescriptor_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVertexBufferDescriptor_getBufferType(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVertexBufferDescriptor_getNormalOffset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVertexBufferDescriptor_getNormalStride(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVertexBufferDescriptor_getVertexOffset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVertexBufferDescriptor_getVertexStride(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVertexBufferDescriptor_hasNormals(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btVertexBufferDescriptor_hasVertexPositions(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btWheelInfo_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* btWheelInfo_new(btWheelInfoConstructionInfo^ ci);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btWheelInfo_getSuspensionRestLength(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void btWheelInfo_updateWheel(void* obj, btRigidBody^ chassis, RaycastInfo^ raycastInfo);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* CastResult_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CastResult_DebugDraw(void* obj, int fraction);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CastResult_drawCoordSystem(void* obj, int^ trans);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CastResult_reportFailure(void* obj, int errNo, int numIterations);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ChunkUtils_getOffset(void* obj, int flags);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ChunkUtils_swapInt(void* obj, int inte);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ChunkUtils_swapLong64(void* obj, int lng);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ChunkUtils_swapShort(void* obj, short sht);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CJoint_Prepare(void* obj, int dt, int iterations);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CJoint_Solve(void* obj, int dt, int sor);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CJoint_Terminate(void* obj, int dt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CJoint_Type(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* ClosestConvexResultCallback_new(int^ convexFromWorld, int^ convexToWorld);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ClosestConvexResultCallback_addSingleResult(void* obj, LocalConvexResult^ convexResult, bool normalInWorldSpace);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* ClosestPointInput_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* ClosestRayResultCallback_new(int^ rayFromWorld, int^ rayToWorld);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ClosestRayResultCallback_addSingleResult(void* obj, LocalRayResult^ rayResult, bool normalInWorldSpace);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* Cluster_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* ClusterBase_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ClusterBase_SolveContact(void* obj, int^ res, Body ba, Body bb, CJoint^ joint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CollideCL_RS_Process(void* obj, int^ leaf);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CollideCL_RS_ProcessColObj(void* obj, btSoftBody^ ps, int^ colObWrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CollideCL_SS_Process(void* obj, int^ la, int^ lb);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CollideCL_SS_ProcessSoftSoft(void* obj, btSoftBody^ psa, btSoftBody^ psb);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CollideSDF_RS_DoNode(void* obj, Node^ n);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CollideSDF_RS_Process(void* obj, int^ leaf);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CollideVF_SS_Process(void* obj, int^ lnode, int^ lface);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* CompoundPrimitiveManager_new(CompoundPrimitiveManager^ compound);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* CompoundPrimitiveManager_new(btGImpactCompoundShape^ compoundShape);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* CompoundPrimitiveManager_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CompoundPrimitiveManager_get_primitive_box(void* obj, int prim_index, int^ primbox);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CompoundPrimitiveManager_get_primitive_count(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CompoundPrimitiveManager_get_primitive_triangle(void* obj, int prim_index, btPrimitiveTriangle^ triangle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CompoundPrimitiveManager_is_trimesh(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* ConstraintCfg_new(float dt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* ContactResultCallback_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ContactResultCallback_addSingleResult(void* obj, int^ cp, btCollisionObjectWrapper^ colObj0Wrap, int partId0, int index0, btCollisionObjectWrapper^ colObj1Wrap, int partId1, int index1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ContactResultCallback_needsCollision(void* obj, btBroadphaseProxy^ proxy0);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* ConvexH_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* ConvexH_new(int vertices_size, int edges_size, int facets_size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* ConvexResultCallback_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ConvexResultCallback_addSingleResult(void* obj, LocalConvexResult^ convexResult, bool normalInWorldSpace);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ConvexResultCallback_hasHit(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ConvexResultCallback_needsCollision(void* obj, btBroadphaseProxy^ proxy0);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* CProfileIterator_new(CProfileNode^ start);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_Enter_Child(void* obj, int index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_Enter_Largest_Child(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_Enter_Parent(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_First(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_Get_Current_Name(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_Get_Current_Parent_Name(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_Get_Current_Parent_Total_Calls(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_Get_Current_Parent_Total_Time(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_Get_Current_Total_Calls(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_Get_Current_Total_Time(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_Get_Current_UserPointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_Is_Done(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_Is_Root(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_Next(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileIterator_Set_Current_UserPointer(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileManager_CleanupMemory(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileManager_dumpAll(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileManager_dumpRecursive(void* obj, CProfileIterator^ profileIterator, int spacing);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileManager_Get_Frame_Count_Since_Reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileManager_Get_Iterator(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileManager_Get_Time_Since_Reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileManager_Increment_Frame_Counter(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileManager_Release_Iterator(void* obj, CProfileIterator^ iterator);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileManager_Reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileManager_Start_Profile(void* obj, char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileManager_Stop_Profile(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* CProfileNode_new(char^ name, CProfileNode^ parent);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileNode_Call(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileNode_CleanupMemory(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileNode_Get_Child(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileNode_Get_Name(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileNode_Get_Parent(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileNode_Get_Sibling(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileNode_Get_Sub_Node(void* obj, char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileNode_Get_Total_Calls(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileNode_Get_Total_Time(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileNode_GetUserPointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileNode_Reset(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileNode_Return(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CProfileNode_SetUserPointer(void* obj, void^ ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* CProfileSample_new(char^ name);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, int^ ci, int^ body0Wrap, int^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, int^ ci, int^ body0Wrap, int^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* CreateFunc_new(int^ simplexSolver, btConvexPenetrationDepthSolver^ pdSolver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, int^ ci, int^ body0Wrap, int^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* CreateFunc_new(btConvexPenetrationDepthSolver^ pdSolver);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* CreateFunc_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ col0Wrap, btCollisionObjectWrapper^ col1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, int^ ci, int^ body0Wrap, int^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, int^ ci, int^ body0Wrap, int^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, int^ ci, int^ body0Wrap, int^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void CreateFunc_CreateCollisionAlgorithm(void* obj, int^ ci, int^ body0Wrap, int^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* DefaultColors_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Edge_getNextEdgeOfFace(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Edge_getNextEdgeOfVertex(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Edge_getReverseEdge(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Edge_getSourceVertex(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Edge_getTargetVertex(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Edge_IsMax(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Edge_getNextEdgeOfFace(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Edge_getNextEdgeOfVertex(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Edge_getReverseEdge(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Edge_getSourceVertex(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Edge_getTargetVertex(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* Element_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* EPA_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void EPA_append(void* obj, sList^ list, sFace^ face);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void EPA_bind(void* obj, sFace^ fa, uint ea, sFace^ fb, uint eb);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void EPA_Evaluate(void* obj, GJK^ gjk, int^ guess);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void EPA_expand(void* obj, uint pass, [unexposed type]^ w, sFace^ f, uint e, sHorizon^ horizon);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void EPA_findbest(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void EPA_getedgedist(void* obj, sFace^ face, [unexposed type]^ a, [unexposed type]^ b, int^ dist);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void EPA_Initialize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void EPA_newface(void* obj, [unexposed type]^ a, [unexposed type]^ b, [unexposed type]^ c, bool forced);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void EPA_remove(void* obj, sList^ list, sFace^ face);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_AABB_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_AABB_new(int^ V1, int^ V2, int^ V3);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_AABB_new(int^ V1, int^ V2, int^ V3, int margin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_AABB_new(GIM_AABB^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_AABB_new(GIM_AABB^ other, int margin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* gim_array_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* gim_array_new(uint reservesize);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_at(void* obj, uint i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_at(void* obj, uint i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_back(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_back(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_clear(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_clear_memory(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_clear_range(void* obj, uint start_range);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_destroyData(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_erase(void* obj, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_erase_sorted(void* obj, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_erase_sorted_mem(void* obj, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_front(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_front(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_get_pointer_at(void* obj, uint i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_get_pointer_at(void* obj, uint i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_growingCheck(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_insert(void* obj, [unexposed type]^ obj, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_insert_mem(void* obj, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_max_size(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_operator[](void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_operator[](void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_pointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_pointer(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_pop_back(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_pop_back_mem(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_push_back(void* obj, [unexposed type]^ obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_push_back_mem(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_push_back_memcpy(void* obj, [unexposed type]^ obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_refit(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_reserve(void* obj, uint size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_resize(void* obj, uint size, bool call_constructor, [unexposed type]^ fillData);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_resizeData(void* obj, uint newsize);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_size(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_array_swap(void* obj, uint i, uint j);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* gim_bitset_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* gim_bitset_new(uint bits_count);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_bitset_clear(void* obj, uint bit_index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_bitset_clear_all(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_bitset_get(void* obj, uint bit_index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_bitset_resize(void* obj, uint newsize);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_bitset_set(void* obj, uint bit_index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_bitset_set_all(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_bitset_size(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_BOX_TREE_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GIM_BOX_TREE__build_sub_tree(void* obj, gim_array^ primitive_boxes, uint startIndex, uint endIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GIM_BOX_TREE__calc_splitting_axis(void* obj, gim_array^ primitive_boxes, uint startIndex, uint endIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GIM_BOX_TREE__sort_and_calc_splitting_index(void* obj, gim_array^ primitive_boxes, uint startIndex, uint endIndex, uint splitAxis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GIM_BOX_TREE_build_tree(void* obj, gim_array^ primitive_boxes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_BOX_TREE_NODE_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_BVH_TREE_NODE_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_CONTACT_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_CONTACT_new(GIM_CONTACT^ contact);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_CONTACT_new(int^ point, int^ normal, int depth, int feature1, int feature2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* gim_contact_array_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_contact_array_merge_contacts_unique(void* obj, gim_contact_array^ contacts);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* gim_hash_table_new(uint reserve_size, uint node_size, uint min_hash_table_size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__assign_hash_table_cell(void* obj, uint hashkey);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__clear_table_memory(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__destroy(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__erase_by_index_hash_table(void* obj, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__erase_hash_table(void* obj, uint hashkey);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__erase_sorted(void* obj, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__erase_unsorted(void* obj, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__find_avaliable_cell(void* obj, uint hashkey);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__find_cell(void* obj, uint hashkey);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__insert_hash_table(void* obj, uint hashkey, [unexposed type]^ value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__insert_hash_table_replace(void* obj, uint hashkey, [unexposed type]^ value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__insert_in_pos(void* obj, uint hashkey, [unexposed type]^ value, uint pos);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__insert_sorted(void* obj, uint hashkey, [unexposed type]^ value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__insert_sorted_replace(void* obj, uint hashkey, [unexposed type]^ value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__insert_unsorted(void* obj, uint hashkey, [unexposed type]^ value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__invalidate_keys(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__rehash(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__reserve_table_memory(void* obj, uint newtablesize);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table__resize_table(void* obj, uint newsize);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_check_for_switching_to_hashtable(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_clear(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_erase_by_index(void* obj, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_erase_by_index_unsorted(void* obj, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_erase_by_key(void* obj, uint hashkey);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_find(void* obj, uint hashkey);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_get_key(void* obj, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_get_value(void* obj, uint hashkey);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_get_value_by_index(void* obj, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_insert(void* obj, uint hashkey, [unexposed type]^ element);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_insert_override(void* obj, uint hashkey, [unexposed type]^ element);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_insert_unsorted(void* obj, uint hashkey, [unexposed type]^ element);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_is_hash_table(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_is_sorted(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_operator[](void* obj, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_operator[](void* obj, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_set_sorted(void* obj, bool value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_size(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_sort(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_switch_to_hashtable(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_hash_table_switch_to_sorted_array(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_HASH_TABLE_NODE_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_HASH_TABLE_NODE_new(GIM_HASH_TABLE_NODE^ value);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_HASH_TABLE_NODE_new(uint key, [unexposed type]^ data);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GIM_HASH_TABLE_NODE_operator<(void* obj, GIM_HASH_TABLE_NODE^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GIM_HASH_TABLE_NODE_operator==(void* obj, GIM_HASH_TABLE_NODE^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GIM_HASH_TABLE_NODE_operator>(void* obj, GIM_HASH_TABLE_NODE^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_PAIR_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_PAIR_new(GIM_PAIR^ p);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_PAIR_new(int index1, int index2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* gim_pair_set_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_pair_set_push_pair(void* obj, uint index1, uint index2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void gim_pair_set_push_pair_inv(void* obj, uint index1, uint index2);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GIM_PRIMITIVE_MANAGER_PROTOTYPE_get_primitive_box(void* obj, uint prim_index, GIM_AABB^ primbox);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GIM_PRIMITIVE_MANAGER_PROTOTYPE_get_primitive_count(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GIM_PRIMITIVE_MANAGER_PROTOTYPE_get_primitive_triangle(void* obj, uint prim_index, GIM_TRIANGLE^ triangle);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GIM_PRIMITIVE_MANAGER_PROTOTYPE_is_trimesh(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_RSORT_TOKEN_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_RSORT_TOKEN_new(GIM_RSORT_TOKEN^ rtoken);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GIM_RSORT_TOKEN_operator<(void* obj, GIM_RSORT_TOKEN^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GIM_RSORT_TOKEN_operator>(void* obj, GIM_RSORT_TOKEN^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GIM_RSORT_TOKEN_COMPARATOR_operator()(void* obj, GIM_RSORT_TOKEN^ a, GIM_RSORT_TOKEN^ b);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_TREE_TREE_COLLIDER_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GIM_TRIANGLE_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GJK_new([unexposed type]^ a, [unexposed type]^ b);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GJK_appendvertice(void* obj, sSimplex^ simplex, int^ v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GJK_det(void* obj, int^ a, int^ b, int^ c);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GJK_EncloseOrigin(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GJK_Evaluate(void* obj, MinkowskiDiff^ shapearg, int^ guess);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GJK_getsupport(void* obj, int^ d, sSV^ sv);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GJK_Initialize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GJK_projectorigin(void* obj, int^ a, int^ b, int^ w, uint^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GJK_projectorigin(void* obj, int^ a, int^ b, int^ c, int^ w, uint^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GJK_projectorigin(void* obj, int^ a, int^ b, int^ c, int^ d, int^ w, uint^ m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GJK_removevertice(void* obj, sSimplex^ simplex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GpuSatCollision_new(int ctx, int device, int q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void GpuSatCollision_computeConvexConvexContactsGPUSAT(void* obj, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* GrahamVector3_new(btVector3^ org, int orgIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* HalfEdge_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* HalfEdge_new(short _ea, byte _v, byte _p);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Handle_BT_DECLARE_ALIGNED_ALLOCATOR(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* HullDesc_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* HullDesc_new(HullFlag flag, uint vcount, btVector3^ vertices, uint stride);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullDesc_ClearHullFlag(void* obj, HullFlag flag);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullDesc_HasHullFlag(void* obj, HullFlag flag);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullDesc_SetHullFlag(void* obj, HullFlag flag);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_allocateTriangle(void* obj, int a, int b, int c);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_b2bfix(void* obj, btHullTriangle^ s, btHullTriangle^ t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_BringOutYourDead(void* obj, btVector3^ verts, uint vcount, btVector3^ overts, uint^ ocount, uint^ indices, uint indexcount);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_calchull(void* obj, btVector3^ verts, int verts_count, TUIntArray^ tris_out, int^ tris_count, int vlimit);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_calchullgen(void* obj, btVector3^ verts, int verts_count, int vlimit);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_checkit(void* obj, btHullTriangle^ t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_CleanupVertices(void* obj, uint svcount, btVector3^ svertices, uint stride, uint^ vcount, btVector3^ vertices, float normalepsilon, btVector3^ scale);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_ComputeHull(void* obj, uint vcount, btVector3^ vertices, PHullResult^ result, uint vlimit);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_ConvexHCrop(void* obj, ConvexH^ convex, btPlane^ slice);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_CreateConvexHull(void* obj, HullDesc^ desc, HullResult^ result);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_deAllocateTriangle(void* obj, btHullTriangle^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_extrudable(void* obj, float epsilon);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_extrude(void* obj, btHullTriangle^ t0, int v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_FindSimplex(void* obj, btVector3^ verts, int verts_count, btAlignedObjectArray^ allow);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_ReleaseResult(void* obj, HullResult^ result);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_removeb2b(void* obj, btHullTriangle^ s, btHullTriangle^ t);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void HullLibrary_test_cube(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* HullResult_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void IClone_CloneLeaf(void* obj, b3DbvtNode^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void IClone_CloneLeaf(void* obj, btDbvtNode^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ICollide_AllLeaves(void* obj, b3DbvtNode^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ICollide_Descent(void* obj, b3DbvtNode^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ICollide_Process(void* obj, b3DbvtNode^ , b3DbvtNode^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ICollide_Process(void* obj, b3DbvtNode^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ICollide_Process(void* obj, b3DbvtNode^ n, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ICollide_AllLeaves(void* obj, btDbvtNode^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ICollide_Descent(void* obj, btDbvtNode^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ICollide_Process(void* obj, btDbvtNode^ , btDbvtNode^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ICollide_Process(void* obj, btDbvtNode^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ICollide_Process(void* obj, btDbvtNode^ n, int );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void IControl_Default(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void IControl_Prepare(void* obj, AJoint^ );
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void IControl_Speed(void* obj, AJoint^ , int current);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void ImplicitFn_Eval(void* obj, int^ x);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* Impulse_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Impulse_operator-(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Impulse_operator*(void* obj, int x);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void InertiaData_B3_DECLARE_ALIGNED_ALLOCATOR(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* InitCache_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void InitCache_addBody(void* obj, int body_index, int parent_index, JointType joint_type, vec3^ parent_r_parent_body_ref, mat33^ body_T_parent_ref, vec3^ body_axis_of_motion, int mass, vec3^ body_r_body_com, mat33^ body_I_body, int user_int, void^ user_ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void InitCache_B3_DECLARE_ALIGNED_ALLOCATOR(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void InitCache_buildIndexSets(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void InitCache_getInertiaData(void* obj, int index, InertiaData^ inertia);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void InitCache_getJointData(void* obj, int index, JointData^ joint);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void InitCache_getParentIndexArray(void* obj, int^ parent_index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void InitCache_getUserInt(void* obj, int index, int^ user_int);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void InitCache_getUserPtr(void* obj, int index, void^ user_ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void InitCache_numBodies(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void InitCache_numDoFs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* int4_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* int4_new(int _x, int _y, int _z, int _w);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void int4_operator[](void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void int4_operator[](void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Island_append(void* obj, Island^ other);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void IslandCallback_processIsland(void* obj, btCollisionObject^ bodies, int numBodies, btPersistentManifold^ manifolds, int numManifolds, int islandId);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void IslandCallback_processIsland(void* obj, int^ bodies, int numBodies, int^ manifolds, int numManifolds, btTypedConstraint^ constraints, int numConstraints, int islandId);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void IWriter_Prepare(void* obj, b3DbvtNode^ root, int numnodes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void IWriter_WriteLeaf(void* obj, b3DbvtNode^ , int index, int parent);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void IWriter_WriteNode(void* obj, b3DbvtNode^ , int index, int parent, int child0, int child1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void IWriter_Prepare(void* obj, btDbvtNode^ root, int numnodes);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void IWriter_WriteLeaf(void* obj, btDbvtNode^ , int index, int parent);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void IWriter_WriteNode(void* obj, btDbvtNode^ , int index, int parent, int child0, int child1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* Joint_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Joint_Prepare(void* obj, int dt, int iterations);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Joint_Solve(void* obj, int dt, int sor);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Joint_Terminate(void* obj, int dt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Joint_Type(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void JointData_B3_DECLARE_ALIGNED_ALLOCATOR(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void less_operator()(void* obj, [unexposed type]^ a, [unexposed type]^ b);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void less_operator()(void* obj, [unexposed type]^ a, [unexposed type]^ b);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void LJoint_Prepare(void* obj, int dt, int iterations);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void LJoint_Solve(void* obj, int dt, int sor);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void LJoint_Terminate(void* obj, int dt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void LJoint_Type(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* LocalConvexResult_new(btCollisionObject^ hitCollisionObject, LocalShapeInfo^ localShapeInfo, int^ hitNormalLocal, int^ hitPointLocal, int hitFraction);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* LocalRayResult_new(btCollisionObject^ collisionObject, LocalShapeInfo^ localShapeInfo, int^ hitNormalLocal, int hitFraction);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* mat33_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* mat33_new(btMatrix3x3^ btm);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void mat33_operator()(void* obj, int i, int j);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void mat33_operator()(void* obj, int i, int j);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void mat33_operator=(void* obj, btMatrix3x3^ rhs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* mat3x_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* mat3x_new(mat3x^ rhs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* mat3x_new(int rows, int cols);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void mat3x_operator=(void* obj, mat3x^ rhs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void mat3x_setZero(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* MinkowskiDiff_new([unexposed type]^ a, [unexposed type]^ b);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MinkowskiDiff_EnableMargin(void* obj, bool enable);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MinkowskiDiff_Support(void* obj, int^ d);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MinkowskiDiff_Support(void* obj, int^ d, uint index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MinkowskiDiff_Support0(void* obj, int^ d);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MinkowskiDiff_Support1(void* obj, int^ d);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* MultiBodyImpl_new(int num_bodies_, int num_dofs_);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_addRelativeJacobianComponent(void* obj, RigidBody^ body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_addUserForce(void* obj, int body_index, vec3^ body_force);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_addUserMoment(void* obj, int body_index, vec3^ body_moment);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_B3_DECLARE_ALIGNED_ALLOCATOR(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_bodyNumDoFs(void* obj, JointType^ type);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_calculateInverseDynamics(void* obj, vecx^ q, vecx^ u, vecx^ dot_u, vecx^ joint_forces);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_calculateJacobians(void* obj, vecx^ q, vecx^ u, KinUpdateType type);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_calculateKinematics(void* obj, vecx^ q, vecx^ u, vecx^ dot_u, KinUpdateType type);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_calculateMassMatrix(void* obj, vecx^ q, bool update_kinematics, bool initialize_matrix, bool set_lower_triangular_matrix, matxx^ mass_matrix);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_calculateStaticData(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_clearAllUserForcesAndMoments(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_generateIndexSets(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyAngularAcceleration(void* obj, int body_index, vec3^ world_dot_omega);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyAngularVelocity(void* obj, int body_index, vec3^ world_omega);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyAxisOfMotion(void* obj, int body_index, vec3^ axis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyCoM(void* obj, int body_index, vec3^ world_com);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyDotJacobianRotU(void* obj, int body_index, vec3^ world_dot_jac_rot_u);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyDotJacobianTransU(void* obj, int body_index, vec3^ world_dot_jac_trans_u);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyFirstMassMoment(void* obj, int body_index, vec3^ first_mass_moment);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyFrame(void* obj, int index, vec3^ world_origin, mat33^ body_T_world);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyJacobianRot(void* obj, int body_index, mat3x^ world_jac_rot);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyJacobianTrans(void* obj, int body_index, mat3x^ world_jac_trans);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyLinearAcceleration(void* obj, int body_index, vec3^ world_acceleration);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyLinearVelocity(void* obj, int body_index, vec3^ world_velocity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyLinearVelocityCoM(void* obj, int body_index, vec3^ world_velocity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyMass(void* obj, int body_index, int^ mass);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyOrigin(void* obj, int body_index, vec3^ world_origin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodySecondMassMoment(void* obj, int body_index, mat33^ second_mass_moment);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyTParentRef(void* obj, int body_index, mat33^ T);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getBodyTransform(void* obj, int body_index, mat33^ world_T_body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getDoFOffset(void* obj, int body_index, int^ q_index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getJointType(void* obj, int body_index, JointType^ joint_type);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getJointTypeStr(void* obj, int body_index, char^ joint_type);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getParentIndex(void* obj, int body_index, int^ m_parent_index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getParentRParentBodyRef(void* obj, int body_index, vec3^ r);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getUserInt(void* obj, int body_index, int^ user_int);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_getUserPtr(void* obj, int body_index, void^ user_ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_jointTypeToString(void* obj, JointType^ type);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_printTree(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_printTree(void* obj, int index, int indentation);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_printTreeData(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_setBodyFirstMassMoment(void* obj, int body_index, vec3^ first_mass_moment);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_setBodyMass(void* obj, int body_index, int mass);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_setBodySecondMassMoment(void* obj, int body_index, mat33^ second_mass_moment);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_setGravityInWorldFrame(void* obj, vec3^ gravity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_setUserInt(void* obj, int body_index, int user_int);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyImpl_setUserPtr(void* obj, int body_index, void^ user_ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* MultiBodyTree_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_addBody(void* obj, int body_index, int parent_index, JointType joint_type, vec3^ parent_r_parent_body_ref, mat33^ body_T_parent_ref, vec3^ body_axis_of_motion, int mass, vec3^ body_r_body_com, mat33^ body_I_body, int user_int, void^ user_ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_addUserForce(void* obj, int body_index, vec3^ body_force);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_addUserMoment(void* obj, int body_index, vec3^ body_moment);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_B3_DECLARE_ALIGNED_ALLOCATOR(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_calculateInverseDynamics(void* obj, vecx^ q, vecx^ u, vecx^ dot_u, vecx^ joint_forces);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_calculateJacobians(void* obj, vecx^ q, vecx^ u);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_calculateJacobians(void* obj, vecx^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_calculateKinematics(void* obj, vecx^ q, vecx^ u, vecx^ dot_u);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_calculateMassMatrix(void* obj, vecx^ q, bool update_kinematics, bool initialize_matrix, bool set_lower_triangular_matrix, matxx^ mass_matrix);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_calculateMassMatrix(void* obj, vecx^ q, matxx^ mass_matrix);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_calculatePositionAndVelocityKinematics(void* obj, vecx^ q, vecx^ u);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_calculatePositionKinematics(void* obj, vecx^ q);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_clearAllUserForcesAndMoments(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_finalize(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getAcceptInvalidMassProperties(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyAngularAcceleration(void* obj, int body_index, vec3^ world_dot_omega);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyAngularVelocity(void* obj, int body_index, vec3^ world_omega);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyAxisOfMotion(void* obj, int body_index, vec3^ axis);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyCoM(void* obj, int body_index, vec3^ world_com);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyDotJacobianRotU(void* obj, int body_index, vec3^ world_dot_jac_rot_u);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyDotJacobianTransU(void* obj, int body_index, vec3^ world_dot_jac_trans_u);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyFirstMassMoment(void* obj, int body_index, vec3^ first_mass_moment);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyJacobianRot(void* obj, int body_index, mat3x^ world_jac_rot);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyJacobianTrans(void* obj, int body_index, mat3x^ world_jac_trans);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyLinearAcceleration(void* obj, int body_index, vec3^ world_acceleration);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyLinearVelocity(void* obj, int body_index, vec3^ world_velocity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyLinearVelocityCoM(void* obj, int body_index, vec3^ world_velocity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyMass(void* obj, int body_index, int^ mass);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyOrigin(void* obj, int body_index, vec3^ world_origin);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodySecondMassMoment(void* obj, int body_index, mat33^ second_mass_moment);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyTParentRef(void* obj, int body_index, mat33^ T);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getBodyTransform(void* obj, int body_index, mat33^ world_T_body);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getDoFOffset(void* obj, int body_index, int^ q_offset);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getJointType(void* obj, int body_index, JointType^ joint_type);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getJointTypeStr(void* obj, int body_index, char^ joint_type);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getParentIndex(void* obj, int body_index, int^ parent_index);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getParentRParentBodyRef(void* obj, int body_index, vec3^ r);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getUserInt(void* obj, int body_index, int^ user_int);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_getUserPtr(void* obj, int body_index, void^ user_ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_numBodies(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_numDoFs(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_printTree(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_printTreeData(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_setAcceptInvalidMassParameters(void* obj, bool flag);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_setBodyFirstMassMoment(void* obj, int body_index, vec3^ first_mass_moment);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_setBodyMass(void* obj, int body_index, int mass);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_setBodySecondMassMoment(void* obj, int body_index, mat33^ second_mass_moment);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_setGravityInWorldFrame(void* obj, vec3^ gravity);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_setUserInt(void* obj, int body_index, int user_int);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MultiBodyTree_setUserPtr(void* obj, int body_index, void^ user_ptr);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* MyCallback_new(int^ from, int^ to, int ignorePart, int ignoreTriangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MyCallback_reportHit(void* obj, int^ hitNormalLocal, int hitFraction, int partId, int triangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* MyInternalTriangleIndexCallback_new(int^ colShape, btGImpactMeshShape^ meshShape, int depth);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void MyInternalTriangleIndexCallback_internalProcessTriangleIndex(void* obj, int^ triangle, int partId, int triangleIndex);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* PHullResult_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* RayFromToCaster_new(int^ rayFrom, int^ rayTo, int mxt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void RayFromToCaster_Process(void* obj, int^ leaf);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void RayFromToCaster_rayFromToTriangle(void* obj, int^ rayFrom, int^ rayTo, int^ rayNormalizedDirection, int^ a, int^ b, int^ c, int maxt);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* RayResultCallback_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void RayResultCallback_addSingleResult(void* obj, LocalRayResult^ rayResult, bool normalInWorldSpace);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void RayResultCallback_hasHit(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void RayResultCallback_needsCollision(void* obj, btBroadphaseProxy^ proxy0);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Result_addContactPoint(void* obj, int^ normalOnBInWorld, int^ pointInWorld, int depth);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Result_setShapeIdentifiersA(void* obj, int partId0, int index0);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void Result_setShapeIdentifiersB(void* obj, int partId1, int index1);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void RigidBody_B3_DECLARE_ALIGNED_ALLOCATOR(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* sHorizon_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* sList_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* Specs_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* Specs_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* SphereTriangleDetector_new(btSphereShape^ sphere, btTriangleShape^ triangle, int contactBreakingThreshold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void SphereTriangleDetector_collide(void* obj, int^ sphereCenter, int^ point, int^ resultNormal, int^ depth, int^ timeOfImpact, int contactBreakingThreshold);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void SphereTriangleDetector_facecontains(void* obj, int^ p, int^ vertices, int^ normal);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void SphereTriangleDetector_getClosestPoints(void* obj, int^ input, int^ output, btIDebugDraw^ debugDraw, bool swapResults);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void SphereTriangleDetector_pointInTriangle(void* obj,  vertices, int^ normal, int^ p);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* sStkCLN_new(b3DbvtNode^ n, b3DbvtNode^ p);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* sStkCLN_new(btDbvtNode^ n, btDbvtNode^ p);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* sStkNN_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* sStkNN_new(b3DbvtNode^ na, b3DbvtNode^ nb);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* sStkNN_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* sStkNN_new(btDbvtNode^ na, btDbvtNode^ nb);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* sStkNP_new(b3DbvtNode^ n, uint m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* sStkNP_new(btDbvtNode^ n, uint m);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* sStkNPS_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* sStkNPS_new(b3DbvtNode^ n, uint m, int v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* sStkNPS_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* sStkNPS_new(btDbvtNode^ n, uint m, int v);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void SwappedCreateFunc_CreateCollisionAlgorithm(void* obj, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void SwappedCreateFunc_CreateCollisionAlgorithm(void* obj, btCollisionAlgorithmConstructionInfo^ ci, btCollisionObjectWrapper^ body0Wrap, btCollisionObjectWrapper^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void SwappedCreateFunc_CreateCollisionAlgorithm(void* obj, int^ ci, int^ body0Wrap, int^ body1Wrap);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* TrimeshPrimitiveManager_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* TrimeshPrimitiveManager_new(TrimeshPrimitiveManager^ manager);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* TrimeshPrimitiveManager_new(int^ meshInterface, int part);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void TrimeshPrimitiveManager_get_primitive_count(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void TrimeshPrimitiveManager_is_trimesh(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void TrimeshPrimitiveManager_lock(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void TrimeshPrimitiveManager_unlock(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* vec3_new();
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* vec3_new(btVector3^ btv);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void vec3_operator()(void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void vec3_operator()(void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void vec3_operator=(void* obj, btVector3^ rhs);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void vec3_size(void* obj);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void* vecx_new(int size);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void vecx_operator()(void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void vecx_operator()(void* obj, int i);
		[DllImport(Native.Dll, CallingConvention = Native.Conv)]
		public static extern void vecx_operator=(void* obj, btVectorX^ rhs);
	}
}
