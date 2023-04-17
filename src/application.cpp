#include "application.h"
#include <iostream>

void VectorTransform(const glm::vec3& in1, const glm::mat3x4& in2, glm::vec3& out)
{
	out[0] = glm::dot(in1, glm::vec3(in2[0])) + in2[0][3];
	out[1] = glm::dot(in1, glm::vec3(in2[1])) + in2[1][3];
	out[2] = glm::dot(in1, glm::vec3(in2[2])) + in2[2][3];
}

void QuaternionMatrix(const glm::quat& q, glm::mat3x4& m)
{
	m[0][0] = 1.0f - 2.0f * q[1] * q[1] - 2.0f * q[2] * q[2];
	m[1][0] = 2.0f * q[0] * q[1] + 2.0f * q[3] * q[2];
	m[2][0] = 2.0f * q[0] * q[2] - 2.0f * q[3] * q[1];

	m[0][1] = 2.0f * q[0] * q[1] - 2.0f * q[3] * q[2];
	m[1][1] = 1.0f - 2.0f * q[0] * q[0] - 2.0f * q[2] * q[2];
	m[2][1] = 2.0f * q[1] * q[2] + 2.0f * q[3] * q[0];

	m[0][2] = 2.0f * q[0] * q[2] + 2.0f * q[3] * q[1];
	m[1][2] = 2.0f * q[1] * q[2] - 2.0f * q[3] * q[0];
	m[2][2] = 1.0f - 2.0f * q[0] * q[0] - 2.0f * q[1] * q[1];
}

void R_ConcatTransforms(const glm::mat3x4& in1, const glm::mat3x4& in2, glm::mat3x4& out)
{
	out[0][0] = in1[0][0] * in2[0][0] + in1[0][1] * in2[1][0] + in1[0][2] * in2[2][0];
	out[0][1] = in1[0][0] * in2[0][1] + in1[0][1] * in2[1][1] + in1[0][2] * in2[2][1];
	out[0][2] = in1[0][0] * in2[0][2] + in1[0][1] * in2[1][2] + in1[0][2] * in2[2][2];
	out[0][3] = in1[0][0] * in2[0][3] + in1[0][1] * in2[1][3] + in1[0][2] * in2[2][3] + in1[0][3];
	out[1][0] = in1[1][0] * in2[0][0] + in1[1][1] * in2[1][0] + in1[1][2] * in2[2][0];
	out[1][1] = in1[1][0] * in2[0][1] + in1[1][1] * in2[1][1] + in1[1][2] * in2[2][1];
	out[1][2] = in1[1][0] * in2[0][2] + in1[1][1] * in2[1][2] + in1[1][2] * in2[2][2];
	out[1][3] = in1[1][0] * in2[0][3] + in1[1][1] * in2[1][3] + in1[1][2] * in2[2][3] + in1[1][3];
	out[2][0] = in1[2][0] * in2[0][0] + in1[2][1] * in2[1][0] + in1[2][2] * in2[2][0];
	out[2][1] = in1[2][0] * in2[0][1] + in1[2][1] * in2[1][1] + in1[2][2] * in2[2][1];
	out[2][2] = in1[2][0] * in2[0][2] + in1[2][1] * in2[1][2] + in1[2][2] * in2[2][2];
	out[2][3] = in1[2][0] * in2[0][3] + in1[2][1] * in2[1][3] + in1[2][2] * in2[2][3] + in1[2][3];
}

Application::Application() : Shared::Application("hl_mdl_viewer")
{
	CONSOLE->execute("hud_show_fps 1");

	mCamera = std::make_shared<Graphics::Camera3D>();
	mCamera->setFieldOfView(70.0f);
	mCamera->setWorldUp({ 0.0f, -1.0f, 0.0f });
	mCamera->setYaw(glm::radians(90.0f));
	mCamera->setPosition({ 0.0f, 0.0f, -100.0f });

	mCameraController = std::make_shared<Shared::FirstPersonCameraController>(mCamera);
	mCameraController->setSensivity(1.0f);
	mCameraController->setSpeed(1.0f);

	//

	m_Model = new MDLFile();
	m_Model->loadFromFile("leet.mdl");

	auto header = m_Model->getHeader();

	std::cout << "Bones:" << std::endl;

	for (int i = 0; i < header->numbones; i++)
	{
		auto bone = header->getBone() + i;

		if (std::string(bone->name) == "Bip01 Spine")
		{
			m_Yaw = &bone->value[3];
		}
		else if (std::string(bone->name) == "Bip01 Spine2")
		{
			std::cout << bone->value[3] << " " << bone->value[4] << " " << bone->value[5] << std::endl;

			m_Pitch = &bone->value[5];
		}

		std::cout << bone->name << std::endl;
	}

	//*m_Pitch = glm::radians(45.0f);
		//*m_Yaw = glm::radians(45.0f);

	std::cout << std::endl;
	std::cout << "Sequences:" << std::endl;

	for (int i = 0; i < header->numseq; i++)
	{
		auto seq = header->getSequence()[i];
		std::cout << "-> " << seq.label << std::endl;
	}

	for (int i = 0; i < header->numtextures; i++)
	{
		auto tex = header->getTexture()[i];
		auto image = Graphics::Image(tex.width, tex.height, 4);

		for (int j = 0; j < tex.width * tex.height; j++)
		{
			auto t = *(uint8_t*)((size_t)header + tex.index + j);
			auto pixel = (uint8_t*)(size_t)image.getMemory() + (j * 4);
			pixel[0] = *(uint8_t*)((size_t)header + tex.index + (tex.width * tex.height) + (t * 3) + 0);
			pixel[1] = *(uint8_t*)((size_t)header + tex.index + (tex.width * tex.height) + (t * 3) + 1);
			pixel[2] = *(uint8_t*)((size_t)header + tex.index + (tex.width * tex.height) + (t * 3) + 2);
			pixel[3] = 255;
		}
		mTextures[i] = std::make_shared<skygfx::Texture>(image.getWidth(), image.getHeight(),
			skygfx::Format::Byte4, image.getMemory());
	}

	std::cout << std::endl;
	std::cout << "Skinrefs:" << std::endl;

	for (int i = 0; i < header->numskinref; i++)
	{
		auto ref = header->getSkinref()[i];
		std::cout << "-> " << ref << std::endl;
	}

	std::cout << std::endl;
	std::cout << "Attachments:" << std::endl;

	for (int i = 0; i < header->numattachments; i++)
	{
		auto attachment = header->getAttachment()[i];
		auto bone = header->getBone()[attachment.bone];

		std::cout << "-> " << attachment.name << ", Bone: " << bone.name << std::endl;
	}

	std::cout << header->name << std::endl;

	for (int i = 0; i < header->numbodyparts; i++)
	{
		auto bodypart = header->getBodypart()[i];

		std::cout << bodypart.name << std::endl;

		m_Bodyparts.push_back(true);
	}

	// build grid linelist

	{
		const glm::vec2 Dimensions = { 1000.0f, 1000.0f };
		const glm::vec4 Color = { Graphics::Color::White, 0.5f };
		const float Step = 32.0f;
		const float Y = 36.0f;

		for (float x = -Dimensions.x; x <= Dimensions.x; x += Step)
		{
			for (float y = -Dimensions.y; y <= Dimensions.y; y += Step)
			{
				mGridLineList.push_back({ { x, Y, -Dimensions.x }, Color });
				mGridLineList.push_back({ { x, Y, Dimensions.x }, Color });
				mGridLineList.push_back({ { -Dimensions.y, Y, y }, Color });
				mGridLineList.push_back({ { Dimensions.y, Y, y }, Color });
			}
		}
	}
}

void Application::onFrame()
{
	showMenu();

	mCamera->onFrame();

	auto view = mCamera->getViewMatrix();
	auto projection = mCamera->getProjectionMatrix();
	auto model = glm::mat4(1.0f);

	model = glm::rotate(model, glm::radians(90.0f), { 1.0f, 0.0f, 0.0f });

	mShader->setProjectionMatrix(projection);
	mShader->setViewMatrix(view);
	mShader->setModelMatrix(model);

	RENDERER->setTopology(skygfx::Topology::TriangleList);
	RENDERER->setViewport(std::nullopt);
	RENDERER->setScissor(std::nullopt);
	RENDERER->setSampler(skygfx::Sampler::Linear);
	RENDERER->setCullMode(skygfx::CullMode::None);
	RENDERER->setDepthMode(skygfx::DepthMode(skygfx::ComparisonFunc::Less));
	RENDERER->setShader(mShader);

	SetUpBones();

	auto header = m_Model->getHeader();

	for (int i = 0; i < header->numbodyparts; i++)
	{
		auto bodypart = header->getBodypart()[i];

		for (int j = 0; j < bodypart.nummodels; j++)
		{
			auto model = bodypart.getModel(header)[j];

			for (int k = 0; k < model.numverts; k++)
			{
				auto pos = model.getVertex(header)[k];
				auto mtx = mBoneMatrices[model.getVertexBoneIndex(header)[k]];
				VectorTransform(pos, mtx, mVertices[k]);
			//	mVertices[k] = mtx * glm::vec3(pos.x, pos.y, pos.z);
			}

			static std::vector<Vertex> vao = {};

			for (int k = 0; k < model.nummesh; k++)
			{
				auto mesh = model.getMesh(header)[k];
				auto tex = header->getSkinref()[mesh.skinref];

				RENDERER->setTexture(*mTextures.at(tex));

				auto texture = header->getTexture()[tex];

				float s = 1.0f / (float)texture.width;
				float t = 1.0f / (float)texture.height;

				auto tris = mesh.getTri(header);

				int l = 0;

				while (l = *(tris++))
				{
					bool fan = l < 0;

					if (fan)
						l = -l;

					vao.clear();

					for (; l > 0; l--, tris += 4)
					{
						Vertex v;

						v.pos = mVertices[tris[0]];
						v.normal = model.getNormal(header)[tris[1]];
						v.texcoord.x = tris[2] * s;
						v.texcoord.y = tris[3] * t;

						if (vao.size() >= 3) // make triangulation 
						{
							if (fan)
							{
								vao.push_back(vao[0]);
								vao.push_back(vao[vao.size() - 2]);
							}
							else
							{
								if (vao.size() & 1)
								{
									vao.push_back(vao[vao.size() - 1]);
									vao.push_back(vao[vao.size() - 3]);
								}
								else
								{
									vao.push_back(vao[vao.size() - 4]);
									vao.push_back(vao[vao.size() - 2]);
								}
							}
						}

						vao.push_back(v);
					}

					RENDERER->setVertexBuffer(vao);
					RENDERER->draw(vao.size());
				}
			}
		}
	}

	// draw grid

	GRAPHICS->begin();
	GRAPHICS->pushDepthMode(skygfx::DepthMode(skygfx::ComparisonFunc::Less));
	GRAPHICS->pushViewMatrix(mCamera->getViewMatrix());
	GRAPHICS->pushProjectionMatrix(mCamera->getProjectionMatrix());
	GRAPHICS->draw(skygfx::Topology::LineList, mGridLineList);
	GRAPHICS->pop(3);
	GRAPHICS->end();
}

void Application::showMenu()
{
	ImGui::SetNextWindowSizeConstraints(ImVec2(0, 0), ImVec2(-1, PLATFORM->getLogicalHeight() - 20));
	ImGui::Begin("Animations", nullptr, ImGui::User::ImGuiWindowFlags_ControlPanel);
	ImGui::SetWindowPos(ImGui::User::TopRightCorner());

	auto header = m_Model->getHeader();

	for (int i = 0; i < header->numseq; i++)
	{
		if (!ImGui::Selectable(header->getSequence()[i].label, m_Sequence == i))
			continue;

		m_Sequence = i;
	}

	ImGui::End();

	ImGui::Begin("Settings", nullptr, ImGui::User::ImGuiWindowFlags_Overlay & ~ImGuiWindowFlags_NoInputs);
	ImGui::SetWindowPos(ImGui::User::BottomLeftCorner());
	ImGui::SliderFloat("Speed", &mSpeed, 0.0f, 2.0f);
	ImGui::End();
}

void Application::SetUpBones()
{
	int					i;

	mstudiobone_t* pbones;
	mstudioseqdesc_t* pseqdesc;
	mstudioanim_t* panim;

	static glm::vec3		pos[MAXSTUDIOBONES];
	glm::mat3x4	bonematrix;
	static glm::quat		q[MAXSTUDIOBONES];

	static glm::vec3		pos2[MAXSTUDIOBONES];
	static glm::vec4		q2[MAXSTUDIOBONES];
	static glm::vec3		pos3[MAXSTUDIOBONES];
	static glm::vec4		q3[MAXSTUDIOBONES];
	static glm::vec3		pos4[MAXSTUDIOBONES];
	static glm::vec4		q4[MAXSTUDIOBONES];

	auto header = m_Model->getHeader();

	pseqdesc = header->getSequence() + m_Sequence;

	m_frame += Clock::ToSeconds(FRAME->getTimeDelta()) * pseqdesc->fps * mSpeed;

	float maxFrames = (float)pseqdesc->numframes - 1; // TODO: is -1 ok?

	if (m_frame >= maxFrames)
		m_frame -= maxFrames;

	panim = GetAnim(pseqdesc);
	CalcRotations(pos, q, pseqdesc, panim, m_frame);

	/*if (pseqdesc->numblends > 1)
	{
		float				s;

		panim += m_pstudiohdr->numbones;
		CalcRotations(pos2, q2, pseqdesc, panim, m_frame);
		s = m_blending[0] / 255.0;

		SlerpBones(q, pos, q2, pos2, s);

		if (pseqdesc->numblends == 4)
		{
			panim += m_pstudiohdr->numbones;
			CalcRotations(pos3, q3, pseqdesc, panim, m_frame);

			panim += m_pstudiohdr->numbones;
			CalcRotations(pos4, q4, pseqdesc, panim, m_frame);

			s = m_blending[0] / 255.0;
			SlerpBones(q3, pos3, q4, pos4, s);

			s = m_blending[1] / 255.0;
			SlerpBones(q, pos, q3, pos3, s);
		}
	}*/

	pbones = header->getBone();

	for (i = 0; i < header->numbones; i++)
	{
		QuaternionMatrix(q[i], bonematrix);
		bonematrix[0][3] = pos[i][0];
		bonematrix[1][3] = pos[i][1];
		bonematrix[2][3] = pos[i][2];

		if (pbones[i].parent == -1)
		{
			mBoneMatrices[i] = bonematrix;
		}
		else
		{
			R_ConcatTransforms(mBoneMatrices[pbones[i].parent], bonematrix, mBoneMatrices[i]);
			//mBoneMatrices[i] = mBoneMatrices[pbones[i].parent] * bonematrix;
		}
	}
}

mstudioanim_t* Application::GetAnim(mstudioseqdesc_t* pseqdesc)
{
	auto header = m_Model->getHeader();

	auto pseqgroup = header->getSequenceGroup() +pseqdesc->seqgroup;

	//if (pseqdesc->seqgroup == 0)
	{
		return (mstudioanim_t*)((uint8_t*)header + pseqgroup->data + pseqdesc->animindex);
	}

	//return (mstudioanim_t*)((byte*)m_panimhdr[pseqdesc->seqgroup] + pseqdesc->animindex);
}

void Application::CalcRotations(glm::vec3* pos, glm::quat* q, mstudioseqdesc_t* pseqdesc, mstudioanim_t* panim, float f)
{
	int					i;
	int					frame;
	mstudiobone_t* pbone;
	float				s;

	frame = (int)f;
	s = (f - frame);

	// add in programatic controllers

	CalcBoneAdj();

	auto header = m_Model->getHeader();

	pbone = header->getBone();
	for (i = 0; i < header->numbones; i++, pbone++, panim++)
	{
		CalcBoneQuaternion(frame, s, pbone, panim, q[i]);
		CalcBonePosition(frame, s, pbone, panim, pos[i]);
	}

	if (pseqdesc->motiontype & STUDIO_X)
		pos[pseqdesc->motionbone][0] = 0.0;
	if (pseqdesc->motiontype & STUDIO_Y)
		pos[pseqdesc->motionbone][1] = 0.0;
	if (pseqdesc->motiontype & STUDIO_Z)
		pos[pseqdesc->motionbone][2] = 0.0;
}

void Application::CalcBoneQuaternion(int frame, float s, mstudiobone_t* pbone, mstudioanim_t* panim, glm::quat& q)
{
	int					j, k;
	glm::quat q1, q2;
	glm::vec3				angle1, angle2;
	mstudioanimvalue_t* panimvalue;

	for (j = 0; j < 3; j++)
	{
		if (panim->offset[j + 3] == 0)
		{
			angle2[j] = angle1[j] = pbone->value[j + 3]; // default;
		}
		else
		{
			panimvalue = (mstudioanimvalue_t*)((uint8_t*)panim + panim->offset[j + 3]);
			k = frame;
			while (panimvalue->num.total <= k)
			{
				k -= panimvalue->num.total;
				panimvalue += panimvalue->num.valid + 1;
			}
			// Bah, missing blend!
			if (panimvalue->num.valid > k)
			{
				angle1[j] = panimvalue[k + 1].value;

				if (panimvalue->num.valid > k + 1)
				{
					angle2[j] = panimvalue[k + 2].value;
				}
				else
				{
					if (panimvalue->num.total > k + 1)
						angle2[j] = angle1[j];
					else
						angle2[j] = panimvalue[panimvalue->num.valid + 2].value;
				}
			}
			else
			{
				angle1[j] = panimvalue[panimvalue->num.valid].value;
				if (panimvalue->num.total > k + 1)
				{
					angle2[j] = angle1[j];
				}
				else
				{
					angle2[j] = panimvalue[panimvalue->num.valid + 2].value;
				}
			}

			angle1[j] = pbone->value[j + 3] + angle1[j] * pbone->scale[j + 3];
			angle2[j] = pbone->value[j + 3] + angle2[j] * pbone->scale[j + 3];

			/*if (std::string(pbone->name) == "Bip01 Spine" && j == 0)
			{
				angle1[j] = glm::radians(45.0f);
			}*/

		}

		if (pbone->bonecontroller[j + 3] != -1)
		{
			angle1[j] += m_adj[pbone->bonecontroller[j + 3]];
			angle2[j] += m_adj[pbone->bonecontroller[j + 3]];
		}
	}

	if (angle1 != angle2)
	{
		q1 = glm::quat(angle1);
		q2 = glm::quat(angle2);
		q = glm::slerp(q1, q2, s);
	}
	else
	{
		q = glm::quat(glm::vec3(angle1.x, angle1.y, angle1.z));
	}
}

void Application::CalcBonePosition(int frame, float s, mstudiobone_t* pbone, mstudioanim_t* panim, glm::vec3& pos)
{
	mstudioanimvalue_t* panimvalue;

	for (int j = 0; j < 3; j++)
	{
		pos[j] = pbone->value[j]; // default;

		if (panim->offset[j] != 0)
		{
			panimvalue = (mstudioanimvalue_t*)((uint8_t*)panim + panim->offset[j]);

			int k = frame;
			// find span of values that includes the frame we want
			while (panimvalue->num.total <= k)
			{
				k -= panimvalue->num.total;
				panimvalue += panimvalue->num.valid + 1;
			}
			// if we're inside the span
			if (panimvalue->num.valid > k)
			{
				// and there's more data in the span
				if (panimvalue->num.valid > k + 1)
				{
					pos[j] += (panimvalue[k + 1].value * (1.0f - s) + s * panimvalue[k + 2].value) * pbone->scale[j];
				}
				else
				{
					pos[j] += panimvalue[k + 1].value * pbone->scale[j];
				}
			}
			else
			{
				// are we at the end of the repeating values section and there's another section with data?
				if (panimvalue->num.total <= k + 1)
				{
					pos[j] += (panimvalue[panimvalue->num.valid].value * (1.0f - s) + s * panimvalue[panimvalue->num.valid + 2].value) * pbone->scale[j];
				}
				else
				{
					pos[j] += panimvalue[panimvalue->num.valid].value * pbone->scale[j];
				}
			}
		}
		if (pbone->bonecontroller[j] != -1)
		{
			pos[j] += m_adj[pbone->bonecontroller[j]];
		}
	}
}

void Application::CalcBoneAdj()
{
	int					i, j;
	float				value;
	mstudiobonecontroller_t* pbonecontroller;

	auto header = m_Model->getHeader();

	pbonecontroller = header->getBoneController();

	for (j = 0; j < header->numbonecontrollers; j++)
	{
		i = pbonecontroller[j].index;

		if (i <= 3)
		{
			// check for 360% wrapping
			if (pbonecontroller[j].type & STUDIO_RLOOP)
			{
				value = 0.0f/*m_controller[i]*/ * (360.0f / 256.0f) + pbonecontroller[j].start;
			}
			else
			{
				value = 0.0f/*m_controller[i]*/ / 255.0f;
				if (value < 0) value = 0;
				if (value > 1.0f) value = 1.0f;
				value = (1.0f - value) * pbonecontroller[j].start + value * pbonecontroller[j].end;
			}
			// Con_DPrintf( "%d %d %f : %f\n", m_controller[j], m_prevcontroller[j], value, dadt );
		}
		else
		{
			value = m_mouth / 64.0f;
			if (value > 1.0f) value = 1.0f;
			value = (1.0f - value) * pbonecontroller[j].start + value * pbonecontroller[j].end;
			// Con_DPrintf("%d %f\n", mouthopen, value );
		}
		switch (pbonecontroller[j].type & STUDIO_TYPES)
		{
		case STUDIO_XR:
		case STUDIO_YR:
		case STUDIO_ZR:
			m_adj[j] = value * (glm::pi<float>() / 180.0f);
			break;
		case STUDIO_X:
		case STUDIO_Y:
		case STUDIO_Z:
			m_adj[j] = value;
			break;
		}
	}
}
