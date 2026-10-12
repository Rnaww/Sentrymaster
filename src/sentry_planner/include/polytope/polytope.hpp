#ifndef _GEOMETYR_UTILS_POLYTOPE_
#define _GEOMETYR_UTILS_POLYTOPE_

#include "Eigen/Dense"
#include "vector"
#include "list"
#include "common_type_name.hpp"
#include "ellipsoid.hpp"
#include "line_segment.hpp"
// #include "geo_utils.hpp"

class Polytope {
private:
    bool undefined{true};
    bool is_known_free{false};
    MatD4f planes;
    bool have_seed_line{false};
public:
    double overlap_depth_with_last_one{0};
    Vec3f interior_pt_with_last_one;
    Ellipsoid ellipsoid_;
    std::pair<Vec3f, Vec3f> seed_line;
    double robot_r;

    Polytope() {}

    Polytope(MatD4f _planes) {
        planes = _planes;
        undefined = false;
    }

    bool HaveSeedLine() {
        return have_seed_line;
    }

    void SetSeedLine(const std::pair<Vec3f, Vec3f> &_seed_line, double r = 0) {
        robot_r = r;
        seed_line = _seed_line;
        have_seed_line = true;
    }

    int SurfNum() {
        if (undefined) {
            return 0;
        }
        return planes.rows();
    }

    Polytope CrossWith(Polytope &b) {
        MatD4f curIH;
        curIH.resize(this->SurfNum() + b.SurfNum(), 4);
        curIH << this->planes, b.GetPlanes();
        Polytope out;
        out.SetPlanes(curIH);
        return out;
    }

    Vec3f CrossCenter(Polytope &b) {
        MatD4f curIH;
        curIH.resize(this->SurfNum() + b.SurfNum(), 4);
        curIH << this->planes, b.GetPlanes();
        Mat3Df curIV; // 走廊的顶点
        if (!geo_utils::enumerateVs(curIH, curIV)) {
            print(fg(color::gold), " -- [processCorridor] Failed to get Overlap enumerateVs .\n");
            return Vec3f(-999, -999, -999);
        }
        double x = (curIV.row(0).maxCoeff() + curIV.row(0).minCoeff()) * 0.5;
        double y = (curIV.row(1).maxCoeff() + curIV.row(1).minCoeff()) * 0.5;
        double z = (curIV.row(2).maxCoeff() + curIV.row(2).minCoeff()) * 0.5;
        return Vec3f(x, y, z);
    }

    bool HaveOverlapWith(Polytope cmp, double eps = 1e-6) {
        return geo_utils::overlap(this->planes, cmp.GetPlanes(), eps);
    }

    MatD4f GetPlanes() {
        return planes;
    }

    void Reset() {
        undefined = true;
        is_known_free = false;
    }

    bool IsKnownFree() {
        if (undefined) {
            return false;
        }
        return is_known_free;
    }

    void SetKnownFree(bool is_free) {
        is_known_free = is_free;
    }

    void SetPlanes(MatD4f _planes) {
        planes = _planes;
        undefined = false;
    }

    void SetEllipsoid(const Ellipsoid &ellip) {
        ellipsoid_ = ellip;
    }

    bool PointIsInside(const Vec3f &pt, const double margin = 0.01) const {
        if (undefined) {
            return false;
        }
        Eigen::Vector4d pt_e;
        pt_e.head(3) = pt;
        pt_e(3) = 1;
        if ((planes * pt_e).maxCoeff() > margin) {
            return false;
        }
        return true;
    }

    /// Sort a set of points in a plane.
    static vec_E<Vec3f> SortPtsInClockWise(vec_E<Vec3f> &pts, Vec3f normal) {
        Vec3f center(0,0,0);
        for (auto pt: pts) {
            center += pt;
        }
        center = center / pts.size();
        Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Vec3f(0, 0, 1), normal);
        center = q.matrix() * center;
        vector<std::pair<double, Vec3f>> pts_valued;
        pts_valued.resize(pts.size());
        Vec3f temp_p;
        for (int i = 0; i < pts.size(); i++) {
            temp_p = q.matrix() * pts[i];
            double theta = atan2(temp_p(1) - center(1), temp_p(0) - center(0));
            pts_valued[i] = make_pair(theta, pts[i]);
        }

        std::sort(
                pts_valued.begin(), pts_valued.end(),
                [](const std::pair<double, Vec3f> &i,
                   const std::pair<double, Vec3f> &j) { return i.first < j.first; });
        vec_E<Vec3f> pts_sorted(pts_valued.size());
        for (size_t i = 0; i < pts_valued.size(); i++)
            pts_sorted[i] = pts_valued[i].second;
        return pts_sorted;
    }

    

};

typedef vector<Polytope> PolytopeVec;

#endif