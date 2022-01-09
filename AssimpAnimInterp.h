#pragma once

#include "assimp/scene.h"

#include <glm/glm.hpp>

class AssimpAnimInterp
{
public:
	static unsigned int FindPosition(float AnimationTime, const aiNodeAnim* pNodeAnim);

	static unsigned int FindRotation(float AnimationTime, const aiNodeAnim* pNodeAnim);

	static unsigned int FindScaling(float AnimationTime, const aiNodeAnim* pNodeAnim);

	static void CalcInterpolatedPosition(aiVector3D& Out, float AnimationTime, const aiNodeAnim* pNodeAnim);

	static void CalcInterpolatedRotation(aiQuaternion& Out, double AnimationTime, const aiNodeAnim* pNodeAnim);

	static void CalcInterpolatedScaling(aiVector3D& Out, float AnimationTime, const aiNodeAnim* pNodeAnim);


	static 	std::pair<aiQuaternion, aiVector3D> makeAnimationMatrix(aiNodeAnim* channel, float AnimationTime);

	AssimpAnimInterp();
	~AssimpAnimInterp();
};

