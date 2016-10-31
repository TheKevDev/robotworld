#include "MathUtils.hpp"

namespace Utils
{
	/**
	 *
	 */
	/* static */float MathUtils::toRadians( float aDegrees)
	{
		return aDegrees * PI / 180.0;
	}
	/**
	 *
	 */
	/* static */float MathUtils::toDegrees( float aRadian)
	{
		return aRadian * 180.0 / PI;
	}
} //namespace Utils
