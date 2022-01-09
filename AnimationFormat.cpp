// AnimationFormat.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>

#include <assimp/Importer.hpp> // C++ importer interface
#include <assimp/scene.h> // Output data structure
#include <assimp/postprocess.h> // Post processing flags

//#include <Windows.h>

#include <fstream>

#include <string>

#include <vector>
#include <array>
#include <list>
#include <iterator>
#include <functional>
#include <cmath>

#include "glm/glm.hpp"
#include <glm/gtx/quaternion.hpp>

#include "AssimpAnimInterp.h"


struct CubicPolynomial {
	double c0, c1, c2, c3;

	double eval(double t)
	{
		double t_squared = t * t;
		double t_cubed = t * t * t;
		return c0 + (c1 * t) + (c2 * t_squared) + (c3 * t_cubed);
	}

	void InitCubicPoly(double p_0, double p_1, double dp_0, double dp_1)
	{
		c0 = p_0;
		c1 = dp_0;
		c2 = (-3.0f * p_0) + (3 * p_1) - (2 * dp_0) - dp_1;
		c3 = (2 * p_0) - (2 * p_1) + dp_0 + dp_1;
	}

	void InitNonuniformCatmullRom(double x0, double x1, double x2, double x3,
		double dt0, double dt1, double dt2)
	{
		// compute tangents when parameterized in [t1,t2]
		double t1 = (x1 - x0) / dt0 - (x2 - x0) / (dt0 + dt1) + (x2 - x1) / dt1;
		double t2 = (x2 - x1) / dt1 - (x3 - x1) / (dt1 + dt2) + (x3 - x2) / dt2;

		// rescale tangents for parametrization in [0,1]
		t1 *= dt1;
		t2 *= dt1;

		InitCubicPoly(x1, x2, t1, t2);
	}
};

struct qKey
{

	char channel_id;
	unsigned int time_id;
	double time;
	unsigned int type;
	double w;
	double x;
	double y;
	double z;
	
	size_t animationIndex;

	qKey() {
		time = 0;
		channel_id = 0;
		type = 0b00;
		w = 0;
		x = 0;
		y = 0;
		z = 0;
	}

	qKey(aiQuatKey key, uint32_t channel) {
		key.mValue.Normalize();
		time = key.mTime;
		channel_id = channel;
		type = 0b00;
		w = key.mValue.w;
		x = key.mValue.x;
		y = key.mValue.y;
		z = key.mValue.z;
	}

	qKey(aiQuatKey key, uint32_t channel, double timeOverride) {
		key.mValue.Normalize();
		time = timeOverride;
		channel_id = channel;
		type = 0b00;
		w = key.mValue.w;
		x = key.mValue.x;
		y = key.mValue.y;
		z = key.mValue.z;
	}


	qKey& operator=(const qKey& key) // copy assignment
	{
		time = key.time;
		channel_id = key.channel_id;
		time_id = key.time_id;
		type = key.type;
		w = key.w;
		x = key.x;
		y = key.y;
		z = key.z;
		return *this;
	}

	struct qbits {
		unsigned int type : 2;
		unsigned int channel : 7;
		unsigned int time : 20;
		unsigned int c0sign : 1;
		unsigned int c1sign : 1;
		unsigned int c2sign : 1;
		//--32
		unsigned int nullc : 2;
		unsigned int x : 20;
		unsigned int y0 : 10;
		//--32
		unsigned int reserved : 1;
		unsigned int nullsign : 1;
		unsigned int y1 : 10;
		unsigned int z : 20;
		//--32

		qKey bitsToKey(double animationDuration) {
			qKey key;
			double & t = key.time;

			t = (static_cast<double>(time) / 0xFFFFF) * animationDuration;

			long double component_range = 1.0L / std::sqrt(2.0L);
			auto dequantize20bcomponent = [](uint32_t comp, long double component_range)->long double {
				return (static_cast<long double>(comp) / 0xFFFFF) * component_range;
			};
			long double c0, c1, c2, restoredcomponent;
			c0 = dequantize20bcomponent(x, component_range)
				* std::pow(-1.0L, static_cast<long double>(c0sign));
			c1 = dequantize20bcomponent((y0 << 10) | y1, component_range)
				* std::pow(-1.0L, static_cast<long double>(c1sign));
			c2 = dequantize20bcomponent(z, component_range)
				* std::pow(-1.0L, static_cast<long double>(c2sign));
			restoredcomponent = std::sqrt(1 - c0 * c0 - c1 * c1 - c2 * c2)
				* std::pow(-1.0L, static_cast<long double>(nullsign));

			uint8_t nullcomponent = nullc;
			if (nullcomponent == 0b00) {
				key.w = restoredcomponent;
				key.x = c0;
				key.y = c1;
				key.z = c2;
			}
			else if (nullcomponent == 0b01)	{
				key.w = c0;
				key.x = restoredcomponent;
				key.y = c1;
				key.z = c2;
			}
			else if (nullcomponent == 0b10)	{
				key.w = c0;
				key.x = c1;
				key.y = restoredcomponent;
				key.z = c2;
			}
			else {
				key.w = c0;
				key.x = c1;
				key.y = c2;
				key.z = restoredcomponent;
			} 

			key.channel_id = channel;
			return key;
		}

	};

	
	qbits getbits(double durationSec, double ticksPerSec) {				
		long double component_range = 1.0L / std::sqrt(2.0L);
		auto quantized20b_component = [&component_range](double comp)->unsigned int 
				{ return std::lround(0xFFFFF * (std::abs(comp) / component_range)); };

		auto component_signbit = [](double n)->unsigned int { return (n < 0b0) ? 0b1 : 0b0; };
		auto build = 
			[&](unsigned int nullcomp, double nullCompValue, double comp0, double comp1, double comp2) -> qbits 
			{
				qbits bits;
				bits.type = type;
				bits.channel = this->channel_id;
				bits.time = std::lround(0xFFFFF * (this->time / ticksPerSec / durationSec));
				bits.c0sign = component_signbit(comp0);
				bits.c1sign = component_signbit(comp1);
				bits.c2sign = component_signbit(comp2);
				bits.nullc = nullcomp;
				bits.nullsign = component_signbit(nullCompValue);
				bits.x = quantized20b_component(comp0);
				unsigned int c1 = quantized20b_component(comp1);
				bits.y0 = c1 >> 10;
				bits.y1 = c1 & 0x3FF;
				bits.z = quantized20b_component(comp2);
				return bits;
			};
		double absw = std::abs(w);
		double absx = std::abs(x);
		double absy = std::abs(y);
		double absz = std::abs(z);
		auto maximum = std::max(std::max(absw, absx),std::max(absy, absz));

		if (maximum == absw) 
			return build(0b00, w, x, y, z);
		else if (maximum == absx) 
			return build(0b01, x, w, y, z);
		else if (maximum == absy) 
			return build(0b10, y, w, x, z);
		else
			return build(0b11, z, w, x, y);		
	}
};

struct vKey
{
	char channel_id;
	unsigned int time_id;
	double time;
	unsigned int type;
	float x;
	float y;
	float z;


	vKey() {
		time = 0;
		channel_id = 0;
		type = 0b00;
		x = 0;
		y = 0;
		z = 0;
	}

	vKey(aiVectorKey key, uint32_t channel) {
		time = key.mTime;
		channel_id = channel;
		x = key.mValue.x;
		y = key.mValue.y;
		z = key.mValue.z;
		type = 0b01;
	}

	vKey(aiVectorKey key, uint32_t channel, double timeOverride) {
		time = timeOverride;
		channel_id = channel;
		x = key.mValue.x;
		y = key.mValue.y;
		z = key.mValue.z;
		type = 0b01;
	}

	vKey& operator=(const vKey& key) // copy assignment
	{
		time = key.time;
		channel_id = key.channel_id;
		time_id = key.time_id;
		type = key.type;
		x = key.x;
		y = key.y;
		z = key.z;
		return *this;
	}

	struct vbits {
		unsigned int type : 2;
		unsigned int channel : 7;
		unsigned int time : 20;
		unsigned int c0sign : 1;
		unsigned int c1sign : 1;
		unsigned int c2sign : 1;
		//--32
		unsigned int reserved : 1;
		unsigned int x : 21;
		unsigned int y0 : 10;
		//--32
		unsigned int y1 : 11;
		unsigned int z : 21;
		//--32

		vKey bitsToKey(double animationDuration)
		{
			vKey key;
			
			key.time = (static_cast<float>(time) / 0xFFFFF) * animationDuration;

			float component_range = 300.0f;
			auto dequantize21bcomponent = [](uint32_t comp, float component_range)->float {
				return (static_cast<float>(comp) / 0x1FFFFF) * component_range;
			};
			float c0, c1, c2;
			c0 = dequantize21bcomponent(x, component_range)
				* std::pow(-1.0f, static_cast<float>(c0sign));
			c1 = dequantize21bcomponent((y0 << 10) | y1, component_range)
				* std::pow(-1.0f, static_cast<float>(c1sign));
			c2 = dequantize21bcomponent(z, component_range)
				* std::pow(-1.0f, static_cast<float>(c2sign));

			key.channel_id = channel;
			key.x = c0;
			key.y = c1;
			key.z = c2;
			return key;
		}

	};

	vbits getbits(double durationSec, double ticksPerSec)
	{
		auto quantized21b_component = [](double comp)->unsigned int
			{ return std::lround(0x1FFFFF * (std::abs(comp) / 300)); };
		auto component_signbit = [](double n)->unsigned int 
			{ return (n < 0) ? 0b1 : 0b0; };
		vbits bits;
		bits.type = type;
		bits.channel = this->channel_id;
		bits.time = std::lround(0xFFFFF * (this->time / ticksPerSec / durationSec));
		bits.c0sign = component_signbit(this->x);
		bits.c1sign = component_signbit(this->y);
		bits.c2sign = component_signbit(this->z);
		bits.x = quantized21b_component(this->x);
		unsigned int y = quantized21b_component(this->y);
		bits.y0 = y >> 11;
		bits.y1 = y & 0x7FF;
		bits.z = quantized21b_component(this->z);
		return bits;
	}
};

struct QuaternionInterpolation {
	CubicPolynomial x;
	CubicPolynomial y;
	CubicPolynomial z;
	CubicPolynomial w;

	aiQuaternion eval(double t)
	{
		return aiQuaternion((float)w.eval(t), (float)x.eval(t), (float)y.eval(t), (float)z.eval(t)).Normalize();
	}

	double vecDistSquared(const qKey& p, const qKey& q)
	{
		double dx = q.x - p.x;
		double dy = q.y - p.y;
		double dz = q.z - p.z;
		double dw = q.w - p.w;
		return dx * dx + dy * dy + dz * dz + dw * dw;
	}

	QuaternionInterpolation(const qKey& p0, const qKey& p1,
		const qKey& p2, const qKey& p3)
	{
		double dt0 = pow(vecDistSquared(p0, p1), 0.25f);
		double dt1 = pow(vecDistSquared(p1, p2), 0.25f);
		double dt2 = pow(vecDistSquared(p2, p3), 0.25f);

		// safety check for repeated points
		if (dt1 < 1e-4f)    
			dt1 = 1.0f;
		if (dt0 < 1e-4f)    
			dt0 = dt1;
		if (dt2 < 1e-4f)    
			dt2 = dt1;

		x.InitNonuniformCatmullRom(p0.x, p1.x, p2.x, p3.x, dt0, dt1, dt2);
		y.InitNonuniformCatmullRom(p0.y, p1.y, p2.y, p3.y, dt0, dt1, dt2);
		z.InitNonuniformCatmullRom(p0.z, p1.z, p2.z, p3.z, dt0, dt1, dt2);
		w.InitNonuniformCatmullRom(p0.w, p1.w, p2.w, p3.w, dt0, dt1, dt2);
	}
};

struct Vector3Interpolation {
	CubicPolynomial x;
	CubicPolynomial y;
	CubicPolynomial z;

	aiVector3D eval(double t)
	{
		return aiVector3D((float)x.eval(t), (float)y.eval(t), (float)z.eval(t));
	}

	double vecDistSquared(const vKey& p, const vKey& q)
	{
		double dx = q.x - p.x;
		double dy = q.y - p.y;
		double dz = q.z - p.z;
		return dx * dx + dy * dy + dz * dz;
	}

	Vector3Interpolation(const vKey& p0, const vKey& p1,
		const vKey& p2, const vKey& p3)
	{
		double dt0 = pow(vecDistSquared(p0, p1), 0.25f);
		double dt1 = pow(vecDistSquared(p1, p2), 0.25f);
		double dt2 = pow(vecDistSquared(p2, p3), 0.25f);

		// safety check for repeated points
		if (dt1 < 1e-4f)    dt1 = 1.0f;
		if (dt0 < 1e-4f)    dt0 = dt1;
		if (dt2 < 1e-4f)    dt2 = dt1;

		x.InitNonuniformCatmullRom(p0.x, p1.x, p2.x, p3.x, dt0, dt1, dt2);
		y.InitNonuniformCatmullRom(p0.y, p1.y, p2.y, p3.y, dt0, dt1, dt2);
		z.InitNonuniformCatmullRom(p0.z, p1.z, p2.z, p3.z, dt0, dt1, dt2);
	}
};

std::list<qKey> generateSplinePointsQuat(aiNodeAnim* keys, size_t keyCount,  uint32_t channel, aiAnimation* anim)
{
#ifdef _DEBUG
	struct plot {
		std::vector<float> w;
		std::vector<float> x;
		std::vector<float> y;
		std::vector<float> z;
		float* p_w;
		float* p_x;
		float* p_y;
		float* p_z;

		void update(float w, float x, float y, float z) 
		{
			this->w.push_back(w);
			this->x.push_back(x);
			this->y.push_back(y);
			this->z.push_back(z);
			p_w = this->w.data();
			p_x = this->x.data();
			p_y = this->y.data();
			p_z = this->z.data();
		}

		void clear() {
			w.clear();
			x.clear();
			y.clear();
			z.clear();
		}
	};

	plot interp_plot;
	plot sample_plot;
	int L;
#endif // DEBUG
	std::list<qKey> splinePoints;

	auto setTimeIDs = [&splinePoints]() {
		auto P = splinePoints.begin();
		P->time_id = 0;
		(++P)->time_id = 0;
		(++P)->time_id = 0;
		(++P)->time_id = 0;
		for (auto it = (++P); it != splinePoints.end(); it++)
			it->time_id = -1;
	};

	splinePoints.insert(splinePoints.end(), qKey(keys->mRotationKeys[0], channel));
	splinePoints.insert(splinePoints.end(), qKey(keys->mRotationKeys[0], channel));	
	if (keys->mNumRotationKeys < 2)
	{
		splinePoints.insert(splinePoints.end(), qKey(keys->mRotationKeys[0], channel, anim->mDuration));
		splinePoints.insert(splinePoints.end(), qKey(keys->mRotationKeys[0], channel, anim->mDuration));
		setTimeIDs();
		return splinePoints;
	}
	else
	{
		splinePoints.insert(splinePoints.end(), qKey(keys->mRotationKeys[keyCount - 1], channel));
		splinePoints.insert(splinePoints.end(), qKey(keys->mRotationKeys[keyCount - 1], channel));
	}
	
	bool insertedPiece = false;	
	double sizeOfLargestDelta = 0;
	double timeOfLargestDelta = 0;
	std::list<qKey>::iterator largestDeltaInsert;
	
	std::list<qKey>::iterator CP0;
	std::list<qKey>::iterator CP1;
	std::list<qKey>::iterator CP2;
	std::list<qKey>::iterator CP3;

	do 
	{
#ifdef _DEBUG
		sample_plot.clear();
		interp_plot.clear();
		L = 0;
#endif // DEBUG
		CP0 = splinePoints.begin();
		CP1 = ++std::list<qKey>::iterator(CP0);
		CP2 = ++std::list<qKey>::iterator(CP1);
		CP3 = ++std::list<qKey>::iterator(CP2);		
		insertedPiece = false;		
		while (std::list<qKey>::iterator(CP3) != splinePoints.end())
		{
			QuaternionInterpolation interp(*CP0, *CP1, *CP2, *CP3);
			
			assert(CP1->time != CP2->time); // todo throw exception, could not achieve target precision (?)
			for (double simTime = CP1->time; simTime < CP2->time; simTime+=0.1)
			{
				//if (std::abs(CP2->time - simTime) < 0.00001) 
					//simTime = CP2->time;
				auto t = (float)(simTime - CP1->time) / (CP2->time - CP1->time); //0-1
				if (std::abs(1 - t) < 0.0000001) { t = 1; }
				assert(t >= 0 && t <= 1);				
				aiQuaternion q_interp = interp.eval(t);	

				aiQuaternion q_sample;
				AssimpAnimInterp::CalcInterpolatedRotation(q_sample, fmod(simTime, anim->mDuration), keys); //todo use hermite interpolation instead of linear?			
				if (q_sample.w < 0) { q_sample = { -q_sample.w, -q_sample.x, -q_sample.y, -q_sample.z }; }				

				double p = q_interp.w*q_sample.w + q_interp.x*q_sample.x + q_interp.y*q_sample.y + q_interp.z*q_sample.z;				
				if (std::abs(std::fmod(p, 1.0f)) < 0.000001) { p = std::round(p); }
				double d_theta = std::acos(2*p*p - 1);
				assert(d_theta == d_theta); //NaN check				
				if (sizeOfLargestDelta < d_theta) {
					sizeOfLargestDelta = d_theta;
					timeOfLargestDelta = simTime;
					largestDeltaInsert = CP2;
				}
#ifdef _DEBUG
				interp_plot.update(q_interp.w, q_interp.x, q_interp.y, q_interp.z);
				sample_plot.update(q_sample.w, q_sample.x, q_sample.y, q_sample.z);
				L++;
#endif // DEBUG				
			}
			CP0++; CP1++; CP2++; CP3++;
		}
		if (sizeOfLargestDelta > 0.015f/*00001f*/)
		{
			aiQuaternion q;
			AssimpAnimInterp::CalcInterpolatedRotation(q, timeOfLargestDelta, keys);
			if (q.w < 0) { q = { -q.w, -q.x, -q.y, -q.z }; }
			aiQuatKey insertKey(timeOfLargestDelta, q);
			splinePoints.insert(largestDeltaInsert,	qKey(insertKey, channel));
			insertedPiece = true;
			sizeOfLargestDelta = 0;
		}
	} while (insertedPiece);

	auto lastCP = --splinePoints.end();
	if (lastCP->time != anim->mDuration)
	{
		lastCP->time = anim->mDuration;
		splinePoints.push_back(*lastCP);
	}

	setTimeIDs();	
	return splinePoints;
}

std::list<vKey> generateSplinePointsVec(aiNodeAnim* keys, size_t keyCount, uint32_t channel, aiAnimation* anim)
{
#ifdef _DEBUG


	struct plot {
		std::vector<float> x;
		std::vector<float> y;
		std::vector<float> z;
		float* p_x;
		float* p_y;
		float* p_z;

		void update(float x, float y, float z)
		{
			this->x.push_back(x);
			this->y.push_back(y);
			this->z.push_back(z);
			p_x = this->x.data();
			p_y = this->y.data();
			p_z = this->z.data();
		}

		void clear() {
			x.clear();
			y.clear();
			z.clear();
		}
	};

	plot interp_plot;
	plot sample_plot;
	int L;
#endif // _DEBUG
	std::list<vKey>  splinePoints;

	auto setTimeIDs = [&splinePoints]() {
		auto P = splinePoints.begin();
		P->time_id = 0;
		(++P)->time_id = 0;
		(++P)->time_id = 0;
		(++P)->time_id = 0;
		for (auto it = (++P); it != splinePoints.end(); it++)
			it->time_id = -1;
	};

	splinePoints.insert(splinePoints.end(), vKey(keys->mPositionKeys[0], channel));
	splinePoints.insert(splinePoints.end(), vKey(keys->mPositionKeys[0], channel));
	if (keys->mNumRotationKeys < 2)
	{
		splinePoints.insert(splinePoints.end(), vKey(keys->mPositionKeys[0], channel, anim->mDuration));
		splinePoints.insert(splinePoints.end(), vKey(keys->mPositionKeys[0], channel, anim->mDuration));
		setTimeIDs();
		return splinePoints;
	}
	else
	{
		splinePoints.insert(splinePoints.end(), vKey(keys->mPositionKeys[keyCount - 1], channel));
		splinePoints.insert(splinePoints.end(), vKey(keys->mPositionKeys[keyCount - 1], channel));
	}

	bool insertedPiece = false;
	double sizeOfLargestDelta = 0;
	double timeOfLargestDelta = 0;
	std::list<vKey>::iterator largestDeltaInsert;

	std::list<vKey>::iterator CP0;
	std::list<vKey>::iterator CP1;
	std::list<vKey>::iterator CP2;
	std::list<vKey>::iterator CP3;
	
	do
	{
#ifdef _DEBUG
		sample_plot.clear();
		interp_plot.clear();
		L = 0;
#endif // DEBUG
		CP0 = splinePoints.begin();
		CP1 = ++std::list<vKey>::iterator(CP0);
		CP2 = ++std::list<vKey>::iterator(CP1);
		CP3 = ++std::list<vKey>::iterator(CP2);
		insertedPiece = false;

		while (std::list<vKey>::iterator(CP3) != splinePoints.end())
		{
			//currentPiece = { 0,0,0 };
			//size_t start = CP1->animationIndex;
			//size_t end = CP2->animationIndex;
			//for (size_t i = start; i < end; i++)
			//{
			Vector3Interpolation interp(*CP0, *CP1, *CP2, *CP3);
			assert(CP1->time != CP2->time);
			for (double simTime = CP1->time; simTime < CP2->time; simTime += 0.1)
			{
				auto t = (float)(simTime - CP1->time) / (CP2->time - CP1->time); //0-1
				if (std::abs(1 - t) < 0.0000001) { t = 1; }
				assert(t >= 0 && t <= 1);
				aiVector3D v_interp = interp.eval(t);

				aiVector3D v_sample;
				AssimpAnimInterp::CalcInterpolatedPosition(v_sample, fmod(simTime, anim->mDuration), keys);


				double d = aiVector3D(v_sample - v_interp).Length();
				if (std::abs(d) < 0.000001)	{ d = 0.0f;	}
				
				if (sizeOfLargestDelta < d)
				{
					sizeOfLargestDelta = d;
					timeOfLargestDelta = simTime;
					largestDeltaInsert = CP2;
				}
#ifdef _DEBUG
				interp_plot.update(v_interp.x, v_interp.y, v_interp.z);
				sample_plot.update(v_sample.x, v_sample.y, v_sample.z);
				L++;
#endif // DEBUG
			}			
			CP0++; CP1++; CP2++; CP3++;
		}
		if (sizeOfLargestDelta > 0.5f)
		{
			aiVector3D v;
			AssimpAnimInterp::CalcInterpolatedPosition(v, timeOfLargestDelta, keys);
			aiVectorKey insertKey(timeOfLargestDelta, v);
			splinePoints.insert(largestDeltaInsert, vKey(insertKey, channel));
			insertedPiece = true;
			sizeOfLargestDelta = 0;
		}
	} while (insertedPiece);

	auto lastCP = --splinePoints.end();
	if (lastCP->time != anim->mDuration)
	{
		lastCP->time = anim->mDuration;
		splinePoints.push_back(*lastCP);
	}

	setTimeIDs();	
	return splinePoints;
}


aiBone* isInMeshBones(aiString name, const aiScene* modelScene) {
	for (unsigned int i = 0; i < modelScene->mNumMeshes; i++) {
		for (size_t j = 0; j < modelScene->mMeshes[i]->mNumBones; j++)
		{
			if (name == modelScene->mMeshes[i]->mBones[j]->mName) {
				return modelScene->mMeshes[i]->mBones[j];
			}
		}
	}
	return nullptr;
}

aiNode* findRootJoint(aiNode* node, const aiScene* scene) {

	if (node->mName == aiString("mixamorig:Hips")) //todo HACK
	{
		return node;
	}

	for (size_t i = 0; i < node->mNumChildren; i++)
	{
		auto result = findRootJoint(node->mChildren[i], scene);
		if (result) {
			return result;
		}
	}
	return nullptr;
}

aiNodeAnim* boneIsInAnimation(aiString name, const aiScene* modelScene) {
	for (size_t j = 0; j < modelScene->mAnimations[0]->mNumChannels; j++)
	{
		if (name == modelScene->mAnimations[0]->mChannels[j]->mNodeName) 
			return modelScene->mAnimations[0]->mChannels[j];
	}
	return nullptr;
}

void flatOrderChannels(aiNode* node, std::vector<aiNodeAnim*>& channels, const aiScene* scene)
{
	int createdEntry = 0;
	//aiBone* boneExists = isInMeshBones(node->mName, scene);
	//if (boneExists)
	if (node->mName != aiString(""))
	{
		aiNodeAnim* animChannelOfBone = boneIsInAnimation(node->mName, scene);
		//assert(animChannelOfBone);
		if (animChannelOfBone) 
			channels.emplace_back(animChannelOfBone);
	}
	for (unsigned int i = 0; (i < node->mNumChildren); i++)
	{
		flatOrderChannels(node->mChildren[i], channels, scene);
	}
}

int main(int argc, char *argv[])
{
	Assimp::Importer modelImporter;
	const aiScene* scene;

	scene = modelImporter.ReadFile(argv[1], aiProcess_CalcTangentSpace 
			| aiProcess_Triangulate | aiProcess_JoinIdenticalVertices 
			| aiProcess_SortByPType | aiProcess_ValidateDataStructure);

	std::vector<aiNodeAnim*>  flatChannels;
	auto rootJoint = findRootJoint(scene->mRootNode, scene);
	flatOrderChannels(rootJoint, flatChannels, scene);
	
	auto & animation = scene->mAnimations[0];		
	
	struct uint8_ring4 {
		uint8_t n;

		uint8_ring4& operator++() {
			n = (n + 1) % 4;
			return *this;
		};

		uint8_t operator+(uint8_t rhs) {
			return (n + rhs) % 4;
		}
	};
	
	int L =0;
	std::vector<uint8_ring4> rotKeyRingIndex = std::vector<uint8_ring4>(flatChannels.size());
	std::vector<uint8_ring4> posKeyRingIndex = std::vector<uint8_ring4>(flatChannels.size());
	std::vector<std::array<qKey, 4>> rotEval= std::vector<std::array<qKey, 4>>(flatChannels.size());
	std::vector<std::array<vKey, 4>> posEval= std::vector<std::array<vKey, 4>>(flatChannels.size());
	std::vector<std::list<qKey>> NO_TID_rotKeys = std::vector<std::list<qKey>>(flatChannels.size());
	std::vector<std::list<vKey>> NO_TID_posKeys = std::vector<std::list<vKey>>(flatChannels.size());
	std::list<qKey> TID_rotKeys;
	std::list<vKey> TID_posKeys;

	for (unsigned int i = 0; i < flatChannels.size(); i++)
	{
		auto & channel = flatChannels[i];
		auto qlist = generateSplinePointsQuat(channel, channel->mNumRotationKeys, i, animation);
		auto vlist = generateSplinePointsVec(channel, channel->mNumPositionKeys, i, animation);

		auto qit = qlist.begin();
		for (size_t j = 0; j < 4; j++)
		{
			rotEval[i][j] = *qit;
			TID_rotKeys.push_back(*qit);
			qit = qlist.erase(qit);
		}

		auto vit = vlist.begin();
		for (size_t j = 0; j < 4; j++)
		{
			posEval[i][j] = *vit;
			TID_posKeys.push_back(*vit);
			vit = vlist.erase(vit);
		}
				
		NO_TID_rotKeys[i] = qlist;
		NO_TID_posKeys[i] = vlist;
				
	}

	auto getQReplaceTime = [&rotKeyRingIndex, &rotEval] (unsigned int i)->double&
	{
		auto & ring = rotKeyRingIndex[i];
		auto & rk = rotEval[i];
		return rk[ring + 2].time;
	};
	auto getVReplaceTime = [&posKeyRingIndex, &posEval]
	(unsigned int i)->double&
	{
		auto & ring = posKeyRingIndex[i];
		auto & rk = posEval[i];
		return rk[ring + 2].time;
	};

	auto insertQKey = [&rotKeyRingIndex, &rotEval] (unsigned int i, qKey & k)
	{
		auto & ring = rotKeyRingIndex[i];
		auto & rk = rotEval[i];
		rk[ring+0] = k;
		++ring;
	};
	auto insertVKey = [&posKeyRingIndex, &posEval]
	(unsigned int i, vKey & k)
	{
		auto & ring = posKeyRingIndex[i];
		auto & rk = posEval[i];
		rk[ring + 0] = k;
		++ring;
	};

	auto rotListSizeSum = [&NO_TID_rotKeys]()->size_t
	{
		size_t sum =0;
		auto it = NO_TID_rotKeys.begin();
		while (it != NO_TID_rotKeys.end())
		{
			sum += it->size();
			it++;
		}
		return sum;
	};
	auto posListSizeSum = [&NO_TID_posKeys]()->size_t
	{
		size_t sum=0;
		auto it = NO_TID_posKeys.begin();
		while (it != NO_TID_posKeys.end())
		{
			sum += it->size();
			it++;
		}
		return sum;
	};

		
	int id = 1;
	while (rotListSizeSum() > 0)
	{
		//find the channel with the nearest replace time
		unsigned int channel = -1;
		double replaceTime = std::numeric_limits<double>::max();
		for (unsigned int i = 0; i < flatChannels.size(); i++)
		{
			auto rt = getQReplaceTime(i);
			if (rt < replaceTime)
			{
				replaceTime = rt;
				channel = i;
			}
		}
		//then update that channel's evaluator, update TID and move.
		auto channelBegin = NO_TID_rotKeys[channel].begin();
		channelBegin->time_id = id;
		id++;
		insertQKey(channel, *channelBegin);
		TID_rotKeys.push_back(*channelBegin);
		NO_TID_rotKeys[channel].erase(channelBegin);
	}

	id = 1;
	while (posListSizeSum() > 0)
	{
		//find the channel with the nearest replace time
		unsigned int channel = -1;
		double replaceTime = std::numeric_limits<double>::max();
		for (unsigned int i = 0; i < flatChannels.size(); i++)
		{
			auto rt = getVReplaceTime(i);
			if (rt < replaceTime)
			{
				replaceTime = rt;
				channel = i;
			}
		}
		//then update that channel's evaluator, update TID and move.
		auto channelBegin = NO_TID_posKeys[channel].begin();
		channelBegin->time_id = id;
		id++;
		insertVKey(channel, *channelBegin);
		TID_posKeys.push_back(*channelBegin);
		NO_TID_posKeys[channel].erase(channelBegin);
	}
		
	double animationTime = 0;
	
	rotKeyRingIndex = std::vector<uint8_ring4>(flatChannels.size());
	posKeyRingIndex = std::vector<uint8_ring4>(flatChannels.size());

	rotEval = std::vector<std::array<qKey, 4>>(flatChannels.size());
	posEval = std::vector<std::array<vKey, 4>>(flatChannels.size());
	
	auto posDataIndex = TID_posKeys.begin();
	auto rotDataIndex = TID_rotKeys.begin();

	/*
	struct plot {
		std::vector<float> w;
		std::vector<float> x;
		std::vector<float> y;
		std::vector<float> z;
		float* p_w;
		float* p_x;
		float* p_y;
		float* p_z;

		void update(float w, float x, float y, float z)
		{
			this->w.push_back(w);
			this->x.push_back(x);
			this->y.push_back(y);
			this->z.push_back(z);
			p_w = this->w.data();
			p_x = this->x.data();
			p_y = this->y.data();
			p_z = this->z.data();
		}

		void clear() {
			w.clear();
			x.clear();
			y.clear();
			z.clear();
		}
	} plotComp, plotUncomp;

	struct plot {
		std::vector<float> x;
		std::vector<float> y;
		std::vector<float> z;
		float* p_x;
		float* p_y;
		float* p_z;

		void update(float x, float y, float z)
		{
			this->x.push_back(x);
			this->y.push_back(y);
			this->z.push_back(z);
			p_x = this->x.data();
			p_y = this->y.data();
			p_z = this->z.data();
		}

		void clear() {
			x.clear();
			y.clear();
			z.clear();
		}
	} plotComp, plotUncomp;

	auto evaluate = [&]()
	{
		for (size_t i = 0; i < flatChannels.size(); i++)
		{
			auto & ring = rotKeyRingIndex[i];
			auto time = animationTime; //todo 0-1
			auto t1 = rotEval[i][ring + 1].time;
			auto t2 = rotEval[i][ring + 2].time;
			auto t = (time - t1) / (t2 - t1); //0-1
			assert(t >= 0 && t <= 1);
		}

		for (size_t i = 0; i < flatChannels.size(); i++)
		{
			auto & ring = posKeyRingIndex[i];
			auto time = animationTime; //todo 0-1
			auto t1 = posEval[i][ring + 1].time;
			auto t2 = posEval[i][ring + 2].time;
			auto t = (time - t1) / (t2 - t1); //
			assert(t >= 0 && t <= 1);
		}
	};

	auto initiateEvaluators = [&]() 
	{
		posDataIndex = TID_posKeys.begin();
		rotDataIndex = TID_rotKeys.begin();
		for (unsigned int i = 0; i < rotEval.size(); i++)
		{
			auto & ring = rotKeyRingIndex[i];
			auto & rk = rotEval[i];
			for (size_t j = 0; j < 4; j++)
			{
				rk[ring + j] = *rotDataIndex;
				rotDataIndex++;
			}
			
		}
		for (unsigned int i = 0; i < posEval.size(); i++)
		{
			auto & ring = posKeyRingIndex[i];
			auto & pk = posEval[i];
			for (size_t j = 0; j < 4; j++)
			{
				pk[ring + j] = *posDataIndex;
				posDataIndex++;
			}
			
		}
		//if (posDataIndex < animationData.posData.size())
		//{
		//	getPosData();
		//}
		//if (rotDataIndex < animationData.rotData.size())
		//{
		//	getRotData();
		//}
	};
	
	auto updateEvaluators = [&](float deltaTime)
	{
		if (animationTime + deltaTime >= animation->mDuration)
		{
			initiateEvaluators();
		}
		animationTime = std::fmod((animationTime + deltaTime), animation->mDuration);

		if (posDataIndex != TID_posKeys.end()) {
			auto channel = posDataIndex->channel_id;
			//auto & pks = posEval[channel];
			//uint8_ring4 & ring = posKeyRingIndex[channel];
			
			while (posEval[channel][posKeyRingIndex[channel] + 2].time < animationTime && posDataIndex != TID_posKeys.end()) {
				// insert new key
				posEval[channel][posKeyRingIndex[channel] + 0] = *posDataIndex;
				++posKeyRingIndex[channel];
				++posDataIndex;

				//get next key if available
				if (posDataIndex != TID_posKeys.end()) {
				
					channel = posDataIndex->channel_id;
				}
			}
		}

		if (rotDataIndex != TID_rotKeys.end())
		{
			auto channel = rotDataIndex->channel_id;
			//std::array<qKey, 4> & rks = rotEval[channel];
			//ring = rotKeyRingIndex[channel];
			//auto tempT = rks[ring + 2].time;


			auto t1 = rotEval[channel][rotKeyRingIndex[channel] + 1].time;
			auto t2 = rotEval[channel][rotKeyRingIndex[channel] + 2].time;
			//assert(t2 - t1 != 0);
			float t = (animationTime - t1) / (t2 - t1); //0-1
			if (!(t >= 0 && t <= 1))
			{
				float t = (animationTime - t1) / (t2 - t1); //0-1
			}
			//assert(t >= 0 && t <= 1);

			while (rotEval[channel][rotKeyRingIndex[channel] + 2].time < animationTime && rotDataIndex != TID_rotKeys.end()) {

				rotEval[channel][rotKeyRingIndex[channel] + 0] = *rotDataIndex;

				++rotKeyRingIndex[channel];
				++rotDataIndex;

				//get next key if available
				if (rotDataIndex != TID_rotKeys.end()) {
					channel = rotDataIndex->channel_id;
				}
			}
		}
	};
	
	initiateEvaluators();
	for (double i = 0; i < animation->mDuration; i += 0.01)
	{
		updateEvaluators(0.01);

		evaluate();
	}
	*/

	std::string filename = argv[1];
	filename = filename.substr(0, filename.find_last_of("."));
	filename.append(".chs");
	std::ofstream file;
	file.open(filename, std::ios::binary | std::ios::trunc);
	if (file.is_open())
	{
		file.seekp(0, std::ios::beg);
		auto durationSec = scene->mAnimations[0]->mDuration/scene->mAnimations[0]->mTicksPerSecond;
		file.write(reinterpret_cast<char*>(&durationSec), sizeof(double));		

		size_t rotKeyCount = TID_rotKeys.size();
		size_t posKeyCount = TID_posKeys.size();
		file.write(reinterpret_cast<char*>(&rotKeyCount), sizeof(size_t));
		file.write(reinterpret_cast<char*>(&posKeyCount), sizeof(size_t));
		for (auto it = TID_rotKeys.begin(); it != TID_rotKeys.end(); it++)
		{
			qKey::qbits bits = it->getbits(durationSec, scene->mAnimations[0]->mTicksPerSecond);
			file.write(reinterpret_cast<char*>(&bits), sizeof(qKey::qbits));
		}
		for (auto it = TID_posKeys.begin(); it != TID_posKeys.end(); it++)
		{
			if (it->channel_id == 0)
			{
				it->z = 0; //animate in place, todo cull earlier for fewer keys?
			}
			vKey::vbits bits = it->getbits(durationSec, scene->mAnimations[0]->mTicksPerSecond);
			file.write(reinterpret_cast<char*>(&bits), sizeof(vKey::vbits));
		}
		file.close();
	}

	filename = "uncompressed.chs";
	file.open(filename, std::ios::binary | std::ios::trunc);
	if (file.is_open())
	{
		file.seekp(0, std::ios::beg);
		auto durationSec = scene->mAnimations[0]->mDuration / scene->mAnimations[0]->mTicksPerSecond;
		file.write(reinterpret_cast<char*>(&durationSec), sizeof(double));


		size_t rotKeyCount = TID_rotKeys.size();
		size_t posKeyCount = TID_posKeys.size();
		file.write(reinterpret_cast<char*>(&rotKeyCount), sizeof(size_t));
		file.write(reinterpret_cast<char*>(&posKeyCount), sizeof(size_t));
		for (auto it = TID_rotKeys.begin(); it != TID_rotKeys.end(); it++)
		{
			qKey bits = *it;
			file.write(reinterpret_cast<char*>(&bits), sizeof(qKey));
		}
		for (auto it = TID_posKeys.begin(); it != TID_posKeys.end(); it++)
		{
			vKey bits = *it;
			file.write(reinterpret_cast<char*>(&bits), sizeof(vKey));
		}
		file.close();
	}

}