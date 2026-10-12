#ifndef _GEOMETRY_UTILS_LINE_SEGMENT_
#define _GEOMETRY_UTILS_LINE_SEGMENT_


/// For utils
#include "common_type_name.hpp"
#include "geo_utils.hpp"

class LineSegment {
 private:
	bool undefined_ = true;
	Vec3f p1_, p2_, center_;
	double length_;
 public:

  LineSegment() {}

  LineSegment(const Vec3f & p1, const Vec3f & p2): p1_(p1),p2_(p2) {
	undefined_ = false;
  }

  bool empty(){
	return undefined_;
  }

  double getLength(){
	length_ = (p1_-p2_).norm();
	return length_;
  }

  Vec3f getCenter(){
	center_ = (p1_+p2_)/2;
	return center_;
  }

};

typedef vector<LineSegment> LineSegments;

#endif