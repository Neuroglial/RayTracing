#include "Render/Filter.h"

namespace RT
{
	Filter::Filter(const PropertyList &props) :
		m_radius(props.getVector2f("Radius", Vec2f(0.5f))),
		m_invRadius(Vec2f(1 / m_radius.x, 1 / m_radius.y)) {}
}

namespace RT
{
	AURORA_REGISTER_CLASS(ABoxFilter, "Box")

		ABoxFilter::ABoxFilter(const PropertyTreeNode& node) : Filter(node.getPropertyList())
	{
		activate();
	}

	Float ABoxFilter::evaluate(const Vec2f& p) const
	{
		return 1.0f;
	}
}