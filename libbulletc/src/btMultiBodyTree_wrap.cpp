#include <BulletInverseDynamics/IDConfig.hpp>
#include "conversion.h"
#include "btMultiBodyTree_wrap.h"




		btInverseDynamics::MultiBodyTree* MultiBodyTree_new(){
			return new btInverseDynamics::MultiBodyTree();
		}
		
		int MultiBodyTree_addBody(btInverseDynamics::MultiBodyTree* obj, int body_index, int parent_index, btInverseDynamics::JointType joint_type, btVector3* parent_r_parent_body_ref,
			btMatrix3x3* body_T_parent_ref, btVector3* body_axis_of_motion, int mass, btVector3* body_r_body_com, btMatrix3x3* body_I_body, int user_int, void* user_ptr){
			btInverseDynamics::vec3 parent_r_parent(*parent_r_parent_body_ref);
			btInverseDynamics::vec3 body_axis_of_mot(*body_axis_of_motion);
			btInverseDynamics::vec3 body_r_body(*body_r_body_com);
			btInverseDynamics::mat33 body_T_parent(*body_T_parent_ref);
			btInverseDynamics::mat33 body_I_bod(*body_I_body);
			return obj->addBody(body_index, parent_index, joint_type, parent_r_parent, body_T_parent, body_axis_of_mot, mass, body_r_body, body_I_bod, user_int,  user_ptr);
		}

		void MultiBodyTree_addUserForce(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* body_force) {
			btInverseDynamics::vec3 body_fr(*body_force);
			obj->addUserForce(body_index, body_fr);
		}

		void MultiBodyTree_addUserMoment(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* body_moment) {
			btInverseDynamics::vec3 body_mom(*body_moment);
			obj->addUserForce(body_index, body_mom);
		}
		
		int MultiBodyTree_calculateInverseDynamics(btInverseDynamics::MultiBodyTree* obj, int numMultiBodyDofs, int baseDofs, btScalar* qq, btScalar* u, btScalar* dot_u, btScalar* joint_forcesOut) {
			btInverseDynamics::vecx nu(numMultiBodyDofs + baseDofs), qdot(numMultiBodyDofs + baseDofs), q(numMultiBodyDofs + baseDofs), joint_force(numMultiBodyDofs + baseDofs);

			for (int i = 0; i < numMultiBodyDofs; i++)
			{
				q[i + baseDofs] = qq[i];
				qdot[i + baseDofs] = u[i];
				nu[i + baseDofs] = dot_u[i];
			}

			// Deal with set gravity the gravity to correspond to the world gravity
			// btInverseDynamics::vec3 id_grav(m_data->m_dynamicsWorld->getGravity());
			int ret = obj->calculateInverseDynamics(q, qdot, nu, &joint_force);
			if (ret != -1)
			{
				for (int i = 0; i < numMultiBodyDofs; i++)
				{
					joint_forcesOut[i] = joint_force[i + baseDofs];
				}
			}
			return ret;
		}
		
		int MultiBodyTree_calculateJacobians(btInverseDynamics::MultiBodyTree* obj, int numMultiBodyDofs, int baseDofs, btScalar* qq) {
			btInverseDynamics::vecx q(numMultiBodyDofs + baseDofs);
			for (int i = 0; i < numMultiBodyDofs; i++)
			{
				q[i + baseDofs] = qq[i];
			}
			int ret = obj->calculateJacobians(q);
			return ret;
		}

		int MultiBodyTree_calculateJacobians2(btInverseDynamics::MultiBodyTree* obj, int numMultiBodyDofs, int baseDofs, btScalar* qq, btScalar* uu) {
			btInverseDynamics::vecx q(numMultiBodyDofs + baseDofs), u(numMultiBodyDofs + baseDofs);
			for (int i = 0; i < numMultiBodyDofs; i++)
			{
				q[i + baseDofs] = qq[i];
				u[i + baseDofs] = uu[i];
			}
			int ret = obj->calculateJacobians(q, u);
			return ret;
		}

		int MultiBodyTree_calculateKinematics(btInverseDynamics::MultiBodyTree* obj, int numMultiBodyDofs, int baseDofs, btScalar* qq, btScalar* uu, btScalar* dot_uu) {
			btInverseDynamics::vecx q(numMultiBodyDofs + baseDofs), u(numMultiBodyDofs + baseDofs), dot_u(numMultiBodyDofs + baseDofs);
			for (int i = 0; i < numMultiBodyDofs; i++)
			{
				q[i + baseDofs] = qq[i];
				u[i + baseDofs] = uu[i];
				dot_u[i + baseDofs] = dot_uu[i];
			}
			int ret = obj->calculateKinematics(q, u, dot_u);
			return ret;
		}

		int MultiBodyTree_calculateMassMatrix(btInverseDynamics::MultiBodyTree* obj, int numMultiBodyDofs, int baseDofs, btScalar* qq, bool update_kinematics, bool initialize_matrix, bool set_lower_triangular_matrix, btScalar* mass_matrix_out) {
			btInverseDynamics::vecx q(numMultiBodyDofs + baseDofs);
			btInverseDynamics::matxx massMatrix(numMultiBodyDofs + baseDofs, numMultiBodyDofs + baseDofs);
			for (int i = 0; i < numMultiBodyDofs; i++)
			{
				q[i + baseDofs] = qq[i];
			}
			int ret = obj->calculateMassMatrix(q, update_kinematics, initialize_matrix, set_lower_triangular_matrix, &massMatrix);
			if (ret != -1)
			{
				for (int i = 0; i < numMultiBodyDofs; i++)
				{
					for (int j = 0; j < numMultiBodyDofs; j++)
					{
						mass_matrix_out[i * numMultiBodyDofs + j] = massMatrix(i + baseDofs, j + baseDofs);
					}
				}
			}
			return ret;
		}

		int MultiBodyTree_calculateMassMatrix2(btInverseDynamics::MultiBodyTree* obj, int numMultiBodyDofs, int baseDofs, btScalar* qq, btScalar* mass_matrix_out) {
			btInverseDynamics::vecx q(numMultiBodyDofs + baseDofs);
			btInverseDynamics::matxx massMatrix(numMultiBodyDofs + baseDofs, numMultiBodyDofs + baseDofs);
			for (int i = 0; i < numMultiBodyDofs; i++)
			{
				q[i + baseDofs] = qq[i];
			}
			int ret = obj->calculateMassMatrix(q, &massMatrix);
			if (ret != -1)
			{
				for (int i = 0; i < numMultiBodyDofs; i++)
				{
					for (int j = 0; j < numMultiBodyDofs; j++)
					{
						mass_matrix_out[i * numMultiBodyDofs + j] = massMatrix(i + baseDofs, j + baseDofs);
					}
				}
			}
			return ret;
		}

		int MultiBodyTree_calculatePositionAndVelocityKinematics(btInverseDynamics::MultiBodyTree* obj, int numMultiBodyDofs, int baseDofs, btScalar* qq, btScalar* uu) {
			btInverseDynamics::vecx q(numMultiBodyDofs + baseDofs), u(numMultiBodyDofs + baseDofs);
			for (int i = 0; i < numMultiBodyDofs; i++)
			{
				q[i + baseDofs] = qq[i];
				u[i + baseDofs] = uu[i];
			}
			int ret = obj->calculatePositionAndVelocityKinematics(q, u);
			return ret;
		}

		int MultiBodyTree_calculatePositionKinematics(btInverseDynamics::MultiBodyTree* obj, int numMultiBodyDofs, int baseDofs, btScalar* qq) {
			btInverseDynamics::vecx q(numMultiBodyDofs + baseDofs);
			for (int i = 0; i < numMultiBodyDofs; i++)
			{
				q[i + baseDofs] = qq[i];
			}
			int ret = obj->calculatePositionKinematics(q);
			return ret;
		}

		int MultiBodyTree_numBodies(btInverseDynamics::MultiBodyTree* obj) {
			return obj->numBodies();
		}

		int MultiBodyTree_numDoFs(btInverseDynamics::MultiBodyTree* obj) {
			return obj->numDoFs();
		}

		void MultiBodyTree_finalize(btInverseDynamics::MultiBodyTree* obj) {
			obj->finalize();
		}

		void MultiBodyTree_delete(btInverseDynamics::MultiBodyTree* obj) {
			delete obj;
		}

		void MultiBodyTree_clearAllUserForcesAndMoments(btInverseDynamics::MultiBodyTree* obj) 
		{
			obj->clearAllUserForcesAndMoments();
		}

		bool MultiBodyTree_getAcceptInvalidMassProperties(btInverseDynamics::MultiBodyTree* obj) 
		{
			return obj->getAcceptInvalidMassProperties();
		}

		int MultiBodyTree_getBodyAngularAcceleration(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3*  world_dot_omega) {
			btInverseDynamics::vec3 tmp;
			int ret = obj->getBodyAngularAcceleration(body_index, &tmp);
			if (ret != -1) {
				BTVECTOR3_SET(world_dot_omega,tmp);
			}
			return ret;
		}

		int MultiBodyTree_getBodyAngularVelocity(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3*  world_omega) {
			btInverseDynamics::vec3 tmp;
			int ret = obj->getBodyAngularVelocity(body_index, &tmp);
			if (ret != -1) {
				BTVECTOR3_SET(world_omega,tmp);
			}
			return ret;
		}

		int MultiBodyTree_getBodyAxisOfMotion(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3*  axis) {
			btInverseDynamics::vec3 tmp;
			int ret = obj->getBodyAxisOfMotion(body_index, &tmp);
			if (ret != -1) {
				BTVECTOR3_SET(axis,tmp);
			}
			return ret;
		}

		int MultiBodyTree_getBodyCoM(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* outVec) {
			btInverseDynamics::vec3 tmp;
			int ret = obj->getBodyCoM(body_index, &tmp);
			if (ret != -1) {
				BTVECTOR3_SET(outVec,tmp);
			}
			return ret;
		}

		int MultiBodyTree_getBodyDotJacobianRotU(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* outVec) {
			btInverseDynamics::vec3 tmp;
			int ret = obj->getBodyDotJacobianRotU(body_index, &tmp);
			if (ret != -1) {
				BTVECTOR3_SET(outVec,tmp);
			}
			return ret;
		}

		int MultiBodyTree_getBodyDotJacobianTransU(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* outVec) {
			btInverseDynamics::vec3 tmp;
			int ret = obj->getBodyDotJacobianTransU(body_index, &tmp);
			if (ret != -1) {
				BTVECTOR3_SET(outVec,tmp);
			}
			return ret;
		}

		int MultiBodyTree_getBodyFirstMassMoment(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* outVec) {
			btInverseDynamics::vec3 tmp;
			int ret = obj->getBodyFirstMassMoment(body_index, &tmp);
			if (ret != -1) {
				BTVECTOR3_SET(outVec,tmp);
			}
			return ret;
		}

		int MultiBodyTree_getBodyJacobianRot(btInverseDynamics::MultiBodyTree* obj, int body_index, int numMultiBodyDofs, int baseDofs, btScalar* world_jac_rot){
			int rows = 3;
			btInverseDynamics::mat3x tmp(rows, numMultiBodyDofs + baseDofs);
			int ret = obj->getBodyJacobianRot(body_index, &tmp);
			if (ret != -1) {
				for (int i = 0; i < rows; i++) {
					for (int j = 0; j < numMultiBodyDofs; j++) {
						world_jac_rot[i * numMultiBodyDofs + j] = tmp(i, j + baseDofs);
					}
				}
			}
			return ret;
		}

		int MultiBodyTree_getBodyJacobianTrans(btInverseDynamics::MultiBodyTree* obj, int body_index, int numMultiBodyDofs, int baseDofs, btScalar* outMat) {
			int rows = 3;
			btInverseDynamics::mat3x tmp(rows, numMultiBodyDofs + baseDofs);
			int ret = obj->getBodyJacobianTrans(body_index, &tmp);
			if (ret != -1) {
				for (int i = 0; i < rows; i++) {
					for (int j = 0; j < numMultiBodyDofs; j++) {
						outMat[i * numMultiBodyDofs + j] = tmp(i, j + baseDofs);
					}
				}
			}
			return ret;
		}

		int MultiBodyTree_getBodyLinearAcceleration(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* outVec) {
			btInverseDynamics::vec3 tmp;
			int ret = obj->getBodyLinearAcceleration(body_index, &tmp);
			if (ret != -1) {
				BTVECTOR3_SET(outVec,tmp);
			}
			return ret;
		}

		int MultiBodyTree_getBodyLinearVelocity(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* outVec) {
			btInverseDynamics::vec3 tmp;
			int ret = obj->getBodyLinearVelocity(body_index, &tmp);
			if (ret != -1) {
				BTVECTOR3_SET(outVec,tmp);
			}
			return ret;
		}

		int MultiBodyTree_getBodyLinearVelocityCoM(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* outVec) {
			btInverseDynamics::vec3 tmp;
			int ret = obj->getBodyLinearVelocityCoM(body_index, &tmp);
			if (ret != -1) {
				BTVECTOR3_SET(outVec,tmp);
			}
			return ret;
		}

		int MultiBodyTree_getBodyMass(btInverseDynamics::MultiBodyTree* obj, int body_index, btScalar* outVal) {
			return obj->getBodyMass(body_index, outVal);
		}

		int MultiBodyTree_getBodyOrigin(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* outVec) {
			btInverseDynamics::vec3 tmp;
			int ret = obj->getBodyOrigin(body_index, &tmp);
			if (ret != -1) {
				BTVECTOR3_SET(outVec,tmp);
			}
			return ret;
		}

		int MultiBodyTree_getBodySecondMassMoment(btInverseDynamics::MultiBodyTree* obj, int body_index, btMatrix3x3* second_mass_moment) {
			btInverseDynamics::mat33 tmp;
			int ret = obj->getBodySecondMassMoment(body_index, &tmp);
			if (ret != -1) {
				BTMATRIX3X3_OUT(second_mass_moment, &tmp);
			}
			return ret;
		}

		int MultiBodyTree_getBodyTParentRef(btInverseDynamics::MultiBodyTree* obj, int body_index, btMatrix3x3* T) {
			btInverseDynamics::mat33 tmp;
			int ret = obj->getBodyTParentRef(body_index, &tmp);
			if (ret != -1) {
				BTMATRIX3X3_OUT(T, &tmp);
			}
			return ret;
		}

		int MultiBodyTree_getBodyTransform(btInverseDynamics::MultiBodyTree* obj, int body_index, btMatrix3x3* world_T_body) {
			btInverseDynamics::mat33 tmp;
			int ret = obj->getBodyTransform(body_index, &tmp);
			if (ret != -1) {
				BTMATRIX3X3_OUT(world_T_body, &tmp);
			}
			return ret;
		}

		int MultiBodyTree_getDoFOffset(btInverseDynamics::MultiBodyTree* obj, int body_index, int* q_offset) {
			return obj->getDoFOffset(body_index, q_offset);
		}

		int MultiBodyTree_getJointType(btInverseDynamics::MultiBodyTree* obj, int body_index, int* joint_type) {
			btInverseDynamics::JointType tmp;
			int ret = obj->getJointType(body_index, &tmp);
			if (ret != -1) {
				*joint_type = (int) tmp;
			}
			return ret;
		}

		int MultiBodyTree_getParentIndex(btInverseDynamics::MultiBodyTree* obj, int body_index, int* parent_index) {
			return obj->getParentIndex(body_index, parent_index);
		}

		int MultiBodyTree_getParentRParentBodyRef(btInverseDynamics::MultiBodyTree* obj, int body_index, btVector3* r) {
			btInverseDynamics::vec3 tmp;
			int ret = obj->getParentRParentBodyRef(body_index, &tmp);
			if (ret != -1) {
				BTVECTOR3_SET(r,tmp);
			}
			return ret;
		}

		int MultiBodyTree_getUserInt(btInverseDynamics::MultiBodyTree* obj, int body_index, int* user_int) {
			return obj->getUserInt(body_index, user_int);
		}

		void* MultiBodyTree_getUserPtr(btInverseDynamics::MultiBodyTree* obj, int body_index) {
			void** outPtr;
			if (-1 == obj->getUserPtr(body_index, outPtr)) {
				return NULL;
			}
			else
			{
				return *outPtr;
			}
		}

		void MultiBodyTree_setAcceptInvalidMassParameters(btInverseDynamics::MultiBodyTree* obj, bool flag) {
			obj->setAcceptInvalidMassParameters(flag);
		}

		int MultiBodyTree_setBodyFirstMassMoment(btInverseDynamics::MultiBodyTree* obj, int body_index, const btVector3* first_mass_moment) {
			btInverseDynamics::vec3 tmp;
			BTVECTOR3_COPY(&tmp, first_mass_moment);
			return obj->setBodyFirstMassMoment(body_index, tmp);
		}

		int MultiBodyTree_setBodyMass(btInverseDynamics::MultiBodyTree* obj, int body_index, btScalar mass) {
			return obj->setBodyMass(body_index, mass);
		}

		int MultiBodyTree_setBodySecondMassMoment(btInverseDynamics::MultiBodyTree* obj, int body_index, const btMatrix3x3* second_mass_moment) {
			btInverseDynamics::mat33 tmp;
			BTMATRIX3X3_SET(&tmp, (btScalar*)second_mass_moment);
			return obj->setBodySecondMassMoment(body_index, tmp);
		}

		void MultiBodyTree_setGravityInWorldFrame(btInverseDynamics::MultiBodyTree* obj, btVector3* gravity) {
			btInverseDynamics::vec3 tmp;
			BTVECTOR3_COPY(&tmp, gravity);
			obj->setGravityInWorldFrame(tmp);
		}

		int MultiBodyTree_setUserInt(btInverseDynamics::MultiBodyTree* obj, int body_index, int user_int) {
			return obj->setUserInt(body_index, user_int);
		}

		int MultiBodyTree_setUserPtr(btInverseDynamics::MultiBodyTree* obj, int body_index, void* user_ptr) {
			return obj->setUserPtr(body_index, user_ptr);
		}