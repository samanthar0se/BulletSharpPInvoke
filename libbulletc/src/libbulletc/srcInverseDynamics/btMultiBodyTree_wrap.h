#include "main.h"
#include "BulletInverseDynamics/IDConfig.hpp"
#include "BulletInverseDynamics/MultiBodyTree.hpp"

#ifdef __cplusplus
extern "C" {
#endif



		EXPORT btInverseDynamics::MultiBodyTree* MultiBodyTree_new();
		EXPORT int MultiBodyTree_addBody(btInverseDynamics::MultiBodyTree* obj, int body_index, int parent_index, btInverseDynamics::JointType joint_type, btVector3* parent_r_parent_body_ref, btMatrix3x3* body_T_parent_ref, btVector3* body_axis_of_motion, int mass, btVector3* body_r_body_com, btMatrix3x3* body_I_body, int user_int, void* user_ptr);
		EXPORT void MultiBodyTree_addUserForce(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* body_force);
		EXPORT void MultiBodyTree_addUserMoment(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* body_moment);
		
		EXPORT int MultiBodyTree_calculateInverseDynamics(btInverseDynamics::MultiBodyTree* obj, int num_dof, int baseDofs, btScalar* q, btScalar* u, btScalar* dot_u, btScalar* joint_forcesOut);
		EXPORT int MultiBodyTree_numBodies(btInverseDynamics::MultiBodyTree* obj);
		EXPORT int MultiBodyTree_numDoFs(btInverseDynamics::MultiBodyTree* obj);
		EXPORT void MultiBodyTree_finalize(btInverseDynamics::MultiBodyTree* obj);
		EXPORT void MultiBodyTree_delete(btInverseDynamics::MultiBodyTree* obj);

		
		EXPORT int MultiBodyTree_calculateJacobians(btInverseDynamics::MultiBodyTree* obj, int num_dof, int baseDofs, btScalar* q);
		EXPORT int MultiBodyTree_calculateJacobians2(btInverseDynamics::MultiBodyTree* obj, int num_dof, int baseDofs, btScalar* q, btScalar* u);
		EXPORT int MultiBodyTree_calculateKinematics(btInverseDynamics::MultiBodyTree* obj, int num_dof, int baseDofs, btScalar* q, btScalar* u, btScalar* dot_u);
		/*
		EXPORT void MultiBodyTree_calculateMassMatrix(btMultiBodyTree* obj, vecx^ q, bool update_kinematics, bool initialize_matrix, bool set_lower_triangular_matrix, matxx^ mass_matrix);
		EXPORT int MultiBodyTree_calculateMassMatrix(btInverseDynamics::MultiBodyTree* obj, int num_dof, int baseDofs, btScalar* q, btScalar* mass_matrix);
		*/
		
		EXPORT int MultiBodyTree_calculatePositionAndVelocityKinematics(btInverseDynamics::MultiBodyTree* obj, int num_dof, int baseDofs, btScalar* q, btScalar* u);
		EXPORT int MultiBodyTree_calculatePositionKinematics(btInverseDynamics::MultiBodyTree* obj, int num_dof, int baseDofs, btScalar* q);
		

		EXPORT void MultiBodyTree_clearAllUserForcesAndMoments(btInverseDynamics::MultiBodyTree* obj);
		EXPORT bool MultiBodyTree_getAcceptInvalidMassProperties(btInverseDynamics::MultiBodyTree* obj);
		EXPORT int MultiBodyTree_getBodyAngularAcceleration(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* world_dot_omega);
		EXPORT int MultiBodyTree_getBodyAngularVelocity(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* world_omega);
		EXPORT int MultiBodyTree_getBodyAxisOfMotion(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* axis);
		EXPORT int MultiBodyTree_getBodyCoM(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* world_com);
		
		EXPORT int MultiBodyTree_getBodyDotJacobianRotU(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* world_dot_jac_rot_u);
		EXPORT int MultiBodyTree_getBodyDotJacobianTransU(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* world_dot_jac_trans_u);
		EXPORT int MultiBodyTree_getBodyFirstMassMoment(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* first_mass_moment);
		//EXPORT int MultiBodyTree_getBodyJacobianRot(btInverseDynamics::MultiBodyTree* obj, int body_index, btMatrix3X* world_jac_rot);
		//EXPORT int MultiBodyTree_getBodyJacobianTrans(btInverseDynamics::MultiBodyTree* obj, int body_index, btMatrix3X* world_jac_trans);
		EXPORT int MultiBodyTree_getBodyLinearAcceleration(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* world_acceleration);
		EXPORT int MultiBodyTree_getBodyLinearVelocity(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* world_velocity);
		EXPORT int MultiBodyTree_getBodyLinearVelocityCoM(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* world_velocity);
		EXPORT int MultiBodyTree_getBodyMass(btInverseDynamics::MultiBodyTree* obj, int body_index, btScalar* mass);
		EXPORT int MultiBodyTree_getBodyOrigin(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* world_origin);
		
		EXPORT int MultiBodyTree_getBodySecondMassMoment(btInverseDynamics::MultiBodyTree* obj, int body_index, btMatrix3x3* second_mass_moment);
		EXPORT int MultiBodyTree_getBodyTParentRef(btInverseDynamics::MultiBodyTree* obj, int body_index, btMatrix3x3* T);
		EXPORT int MultiBodyTree_getBodyTransform(btInverseDynamics::MultiBodyTree* obj, int body_index, btMatrix3x3* world_T_body);
		EXPORT int MultiBodyTree_getDoFOffset(btInverseDynamics::MultiBodyTree* obj, int body_index, int* q_offset);
		EXPORT int MultiBodyTree_getJointType(btInverseDynamics::MultiBodyTree* obj, int body_index, int* joint_type);
		//EXPORT int MultiBodyTree_getJointTypeStr(btInverseDynamics::MultiBodyTree* obj, int body_index, char^ joint_type);
		EXPORT int MultiBodyTree_getParentIndex(btInverseDynamics::MultiBodyTree* obj, int body_index, int* parent_index);
		EXPORT int MultiBodyTree_getParentRParentBodyRef(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* r);
		EXPORT int MultiBodyTree_getUserInt(btInverseDynamics::MultiBodyTree* obj, int body_index, int* user_int);
		EXPORT void* MultiBodyTree_getUserPtr(btInverseDynamics::MultiBodyTree* obj, int body_index);
		
		EXPORT  void MultiBodyTree_setAcceptInvalidMassParameters(btInverseDynamics::MultiBodyTree* obj, bool flag);
		EXPORT int MultiBodyTree_setBodyFirstMassMoment(btInverseDynamics::MultiBodyTree* obj, int body_index, const btVector3* first_mass_moment);
		EXPORT int MultiBodyTree_setBodyMass(btInverseDynamics::MultiBodyTree* obj, int body_index, btScalar mass);
		EXPORT int MultiBodyTree_setBodySecondMassMoment(btInverseDynamics::MultiBodyTree* obj, int body_index, const btMatrix3x3* second_mass_moment);
		EXPORT void MultiBodyTree_setGravityInWorldFrame(btInverseDynamics::MultiBodyTree* obj, btVector3* gravity);
		EXPORT int MultiBodyTree_setUserInt(btInverseDynamics::MultiBodyTree* obj, int body_index, int user_int);
		EXPORT int MultiBodyTree_setUserPtr(btInverseDynamics::MultiBodyTree* obj, int body_index, void* user_ptr);
		
#ifdef __cplusplus
}
#endif
