#pragma once

#include <limits>
#include <memory>

#include "glog/logging.h"

#if defined(_MSC_VER)
#define NOMINMAX
#endif

#define GLM_FORCE_LEFT_HANDED
#include "glm/glm.hpp"

// Platform-specific definitions
#if defined(_WIN32) || defined(_WIN64)
#define AURORA_WINDOWS_OS
#endif

#define ALLOCA(TYPE, COUNT) (TYPE *) alloca((COUNT) * sizeof(TYPE))

namespace RT
{

	//Float type
#ifdef AURORA_DOUBLE_AS_FLOAT
	typedef double Float;
#else
	typedef float Float;
#endif

	using Byte = unsigned char;

	constexpr static Float ShadowEpsilon = 0.0001f;
	constexpr static Float Pi = 3.14159265358979323846f;
	constexpr static Float InvPi = 0.31830988618379067154f;
	constexpr static Float Inv2Pi = 0.15915494309189533577f;
	constexpr static Float Inv4Pi = 0.07957747154594766788f;
	constexpr static Float PiOver2 = 1.57079632679489661923f;
	constexpr static Float PiOver4 = 0.78539816339744830961f;
	constexpr static Float Sqrt2 = 1.41421356237309504880f;

	constexpr static Float MaxFloat = std::numeric_limits<Float>::max();
	constexpr static Float Infinity = std::numeric_limits<Float>::infinity();
	constexpr static Float MachineEpsilon = std::numeric_limits<Float>::epsilon() * 0.5f;

	inline Float lerp(Float t, Float v1, Float v2) { return (1 - t) * v1 + t * v2; }
	inline Float gamma(int n) { return (n * MachineEpsilon) / (1 - n * MachineEpsilon); }

	inline Float gammaCorrect(Float value) 
	{
		if (value <= 0.0031308f)
			return 12.92f * value;
		return 1.055f * glm::pow(value, (Float)(1.f / 2.4f)) - 0.055f;
	}

	inline Float inverseGammaCorrect(Float value) 
	{
		if (value <= 0.04045f) 
			return value * 1.f / 12.92f;
		return glm::pow((value + 0.055f) * 1.f / 1.055f, (Float)2.4f);
	}

	template <typename T, typename U, typename V>
	inline T clamp(T val, U low, V high)
	{
		if (val < low)
			return low;
		else if (val > high)
			return high;
		else
			return val;
	}

	class Ray;
	class Film;
	class BSDF;
	class BxDF;
	class Light;
	class Shape;
	class Scene;
	class Camera;
	class Hitable;
	class HitableObject;
	class FilmTile;
	class Sampler;
	class Material;
	class AreaLight;
	class Transform;
	class Renderer;
	class CameraSample;
	class RGBSpectrum;
	class Interaction;
	class Distribution1D;
	class VisibilityTester;
	class SurfaceInteraction;

	class MemoryArena;

	using Spectrum = RGBSpectrum;

	// TransportMode Declarations
	enum class TransportMode { Radiance, Importance };

	inline uint32_t floatToBits(float f) 
	{
		uint32_t ui;
		memcpy(&ui, &f, sizeof(float));
		return ui;
	}

	inline float bitsToFloat(uint32_t ui) 
	{
		float f;
		memcpy(&f, &ui, sizeof(uint32_t));
		return f;
	}

	inline uint64_t floatToBits(double f) 
	{
		uint64_t ui;
		memcpy(&ui, &f, sizeof(double));
		return ui;
	}

	inline double bitsToFloat(uint64_t ui) 
	{
		double f;
		memcpy(&f, &ui, sizeof(uint64_t));
		return f;
	}

	//-------------------------------------------stringPrintf-------------------------------------

	inline void stringPrintfRecursive(std::string *s, const char *fmt) 
	{
		const char *c = fmt;
		// No args left; make sure there aren't any extra formatting
		// specifiers.
		while (*c) 
		{
			if (*c == '%') 
			{
				CHECK_EQ(c[1], '%');
				++c;
			}
			*s += *c++;
		}
	}

	// 1. Copy from fmt to *s, up to the next formatting directive.
	// 2. Advance fmt past the next formatting directive and return the
	//    formatting directive as a string.
	inline std::string copyToFormatString(const char **fmt_ptr, std::string *s) 
	{
		const char *&fmt = *fmt_ptr;
		while (*fmt) 
		{
			if (*fmt != '%') 
			{
				*s += *fmt;
				++fmt;
			}
			else if (fmt[1] == '%') 
			{
				// "%%"; let it pass through
				*s += '%';
				*s += '%';
				fmt += 2;
			}
			else
				// fmt is at the start of a formatting directive.
				break;
		}

		std::string nextFmt;
		if (*fmt) 
		{
			do 
			{
				nextFmt += *fmt;
				++fmt;
				// Incomplete (but good enough?) test for the end of the
				// formatting directive: a new formatting directive starts, we
				// hit whitespace, or we hit a comma.
			} while (*fmt && *fmt != '%' && !isspace(*fmt) && *fmt != ',' &&
				*fmt != '[' && *fmt != ']' && *fmt != '(' && *fmt != ')');
		}

		return nextFmt;
	}

	template <typename T>
	inline std::string formatOne(const char *fmt, T v) 
	{
		// Figure out how much space we need to allocate; add an extra
		// character for the '\0'.
		size_t size = snprintf(nullptr, 0, fmt, v) + 1;
		std::string str;
		str.resize(size);
		snprintf(&str[0], size, fmt, v);
		str.pop_back();  // remove trailing NUL
		return str;
	}

	// General-purpose version of stringPrintfRecursive; add the formatted
	// output for a single StringPrintf() argument to the final result string
	// in *s.
	template <typename T, typename... Args>
	inline void stringPrintfRecursive(std::string *s, const char *fmt, T v, Args... args) 
	{
		std::string nextFmt = copyToFormatString(&fmt, s);
		*s += formatOne(nextFmt.c_str(), v);
		stringPrintfRecursive(s, fmt, args...);
	}

	// Special case of StringPrintRecursive for float-valued arguments.
	template <typename... Args>
	inline void stringPrintfRecursive(std::string *s, const char *fmt, float v, Args... args) 
	{
		std::string nextFmt = copyToFormatString(&fmt, s);
		if (nextFmt == "%f")
			*s += formatOne("%.9g", v);
		else
			// If a specific formatting string other than "%f" was specified,
			// just use that.
			*s += formatOne(nextFmt.c_str(), v);

		// Go forth and print the next arg.
		stringPrintfRecursive(s, fmt, args...);
	}

	template <typename... Args>
	inline void stringPrintfRecursive(std::string *s, const char *fmt, double v, Args... args) 
	{
		std::string nextFmt = copyToFormatString(&fmt, s);
		if (nextFmt == "%f")
			*s += formatOne("%.17g", v);
		else
			*s += formatOne(nextFmt.c_str(), v);
		stringPrintfRecursive(s, fmt, args...);
	}

	template <typename... Args>
	inline std::string stringPrintf(const char *fmt, Args... args) 
	{
		std::string ret;
		stringPrintfRecursive(&ret, fmt, args...);
		return ret;
	}

}