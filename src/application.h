#pragma once

#include <shared/all.h>
#include <HL/mdlfile.h>

class Application : public Shared::Application, 
	public Common::FrameSystem::Frameable
{
public:
	Application();

private:
	void onFrame() override;
	void showMenu();

private:
	//using Vertex = skygfx::vertex::PositionTextureNormal;
	//std::shared_ptr<Renderer::Shaders::Generic> mShader = std::make_shared<Renderer::Shaders::Generic>(Vertex::Layout);

private:
	std::shared_ptr<Graphics::Camera3D> mCamera;
	std::shared_ptr<Shared::FirstPersonCameraController> mCameraController;

private:
	float mSpeed = 1.0f;
	MDLFile* m_Model;
	float* m_Pitch;
	float* m_Yaw;
	int m_Sequence = 4;
	float m_frame = 0;
	glm::vec4 m_adj;				// FIX: non persistant, make static
	float m_mouth = 0.0f;
	std::vector<bool> m_Bodyparts;
	std::unordered_map<int, std::shared_ptr<skygfx::Texture>> mTextures;
	glm::vec3 mVertices[MAXSTUDIOVERTS];
	glm::mat3x4 mBoneMatrices[MAXSTUDIOBONES];
	std::vector<skygfx::utils::Mesh::Vertex> mGridLineList;

private:
	void SetUpBones();
	mstudioanim_t* GetAnim(mstudioseqdesc_t* pseqdesc);
	void CalcBoneAdj();
	void CalcBonePosition(int frame, float s, mstudiobone_t* pbone, mstudioanim_t* panim, glm::vec3& pos);
	void CalcBoneQuaternion(int frame, float s, mstudiobone_t* pbone, mstudioanim_t* panim, glm::quat& q);
	void CalcRotations(glm::vec3* pos, glm::quat* q, mstudioseqdesc_t* pseqdesc, mstudioanim_t* panim, float f);
};
