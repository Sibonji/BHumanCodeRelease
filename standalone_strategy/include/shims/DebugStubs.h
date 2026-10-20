#pragma once

#include "Debugging/ColorRGBA.h"
#include <string>

#define DECLARE_DEBUG_DRAWING(id, type) static_cast<void>(0)
#define DECLARE_DEBUG_DRAWING3D(id, type) static_cast<void>(0)
#define COMPLEX_DRAWING(id) if(false)
#define DEBUG_DRAWING(id, type) if(false)
#define DEBUG_DRAWING3D(id, type) if(false)
#define DEBUG_RESPONSE(id) if(false)
#define DEBUG_RESPONSE_ONCE(id) if(false)
#define MODIFY(id, data) static_cast<void>(0)
#define MODIFY_ONCE(id, data) static_cast<void>(0)
#define STOPWATCH(...)
#define ANNOTATION(id, text) static_cast<void>(0)
#define OUTPUT_TEXT(text) static_cast<void>(0)
#define OUTPUT(...) static_cast<void>(0)
#define OUTPUT_ERROR(...) static_cast<void>(0)
#define OUTPUT_WARNING(...) static_cast<void>(0)

#define CIRCLE(...) static_cast<void>(0)
#define CIRCLE3D(...) static_cast<void>(0)
#define LINE(...) static_cast<void>(0)
#define LINE3D(...) static_cast<void>(0)
#define CROSS(...) static_cast<void>(0)
#define CROSS3D(...) static_cast<void>(0)
#define RECTANGLE(...) static_cast<void>(0)
#define RECTANGLE2(...) static_cast<void>(0)
#define RECTANGLE3D(...) static_cast<void>(0)
#define ARROW(...) static_cast<void>(0)
#define POLYGON(...) static_cast<void>(0)
#define FILLED_RECTANGLE(...) static_cast<void>(0)
#define DRAW_TEXT(...) static_cast<void>(0)
#define PLOT(...) static_cast<void>(0)
#define PLOT_POLYGON(...) static_cast<void>(0)
#define TRANSLATE3D(...) static_cast<void>(0)
#define ROTATE3D(...) static_cast<void>(0)
#define SPHERE3D(...) static_cast<void>(0)
#define CYLINDER3D(...) static_cast<void>(0)
#define QUAD3D(...) static_cast<void>(0)

#define ARC(...) static_cast<void>(0)
#define LARGE_DOT(...) static_cast<void>(0)
#define RENDER_OPTIONS3D(...) static_cast<void>(0)
#define RING_SECTOR3D(...) static_cast<void>(0)
#define ROBOT(...) static_cast<void>(0)
#define DRAWDIGIT3D(...) static_cast<void>(0)
#define POINT3D(...) static_cast<void>(0)
#define CYLINDER_LINE3D(...) static_cast<void>(0)
#define CUBE3D(...) static_cast<void>(0)
#define COORDINATES3D(...) static_cast<void>(0)
#define SUBCOORDINATES3D(...) static_cast<void>(0)
#define SCALE3D(...) static_cast<void>(0)
#define ELLIPSOID3D(...) static_cast<void>(0)
#define CYLINDER3D2(...) static_cast<void>(0)
#define CYLINDER_ARROW3D(...) static_cast<void>(0)
#define IMAGE3D(...) static_cast<void>(0)
#define FOOT3D(...) static_cast<void>(0)
#define ELLIPSE(...) static_cast<void>(0)
#define DOT(...) static_cast<void>(0)
#define DOT_AS_VECTOR(...) static_cast<void>(0)
#define MID_DOT(...) static_cast<void>(0)
#define RAY(...) static_cast<void>(0)
#define QUADRANGLE(...) static_cast<void>(0)
#define SPOT(...) static_cast<void>(0)
#define TIP(...) static_cast<void>(0)
#define THREAD(...) static_cast<void>(0)
#define ORIGIN(...) static_cast<void>(0)
#define DRAW_ROBOT_POSE(...) static_cast<void>(0)
#define DRAW_ROBOT_POSE_WITH_HEAD_ROTATION(...) static_cast<void>(0)
#define DRAW_ROBOT_POSE_ROTATIONAL_STANDARD_DEVIATION(...) static_cast<void>(0)
#define COVARIANCE_ELLIPSES_2D_OWN_COLORS(...) static_cast<void>(0)
#define COVARIANCE_ELLIPSES_2D(...) static_cast<void>(0)
#define COVARIANCE_ELLIPSE_2D(...) static_cast<void>(0)
#define GRID_MONO(...) static_cast<void>(0)
#define GRID_RGBA(...) static_cast<void>(0)
#define GRID_RECTANGLE_RGBA(...) static_cast<void>(0)

namespace Drawings
{
  enum PenStyle { dashedPen, solidPen, noPen };
  enum BrushStyle { dashedBrush, solidBrush, noBrush };
}

namespace Drawings3D
{
  enum { disableTransparency = 0, disableOpacity = 0, disableDepth = 0 };
}
