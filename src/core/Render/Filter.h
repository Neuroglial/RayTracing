#pragma once

#include "Utils/Base.h"
#include "Utils/Math.h"
#include "Object/Object.h"

namespace RT
{
	class AFilter : public Object
	{
	public:
		
		virtual ~AFilter() = default;

		AFilter(const PropertyList &props);
		AFilter(const Vec2f &radius)
			: m_radius(radius), m_invRadius(Vec2f(1 / radius.x, 1 / radius.y)) {}

		virtual Float evaluate(const Vec2f &p) const = 0;

		virtual AClassType getClassType() const override { return AClassType::AEFilter; }

		const Vec2f m_radius, m_invRadius;
	};

}

namespace RT
{
	class ABoxFilter final : public AFilter
	{
	public:

		ABoxFilter(const PropertyTreeNode& node);
		ABoxFilter(const Vec2f& radius) : AFilter(radius) {}

		virtual Float evaluate(const Vec2f& p) const override;

		virtual std::string toString() const override { return "BoxFilter[]"; }

	};
}