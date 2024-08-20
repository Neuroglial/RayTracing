#pragma once

#include "Utils/Base.h"
#include "Utils/Math.h"
#include "Object/Object.h"

namespace RT
{
	class Filter : public Object
	{
	public:
		
		virtual ~Filter() = default;

		Filter(const PropertyList &props);
		Filter(const Vec2f &radius)
			: m_radius(radius), m_invRadius(Vec2f(1 / radius.x, 1 / radius.y)) {}

		virtual Float evaluate(const Vec2f &p) const = 0;

		virtual ClassType getClassType() const override { return ClassType::RTFilter; }

		const Vec2f m_radius, m_invRadius;
	};

}

namespace RT
{
	class ABoxFilter final : public Filter
	{
	public:

		ABoxFilter(const PropertyTreeNode& node);
		ABoxFilter(const Vec2f& radius) : Filter(radius) {}

		virtual Float evaluate(const Vec2f& p) const override;

		virtual std::string toString() const override { return "BoxFilter[]"; }

	};
}