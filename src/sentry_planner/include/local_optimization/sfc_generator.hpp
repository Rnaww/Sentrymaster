#pragma once
#include "common.h"
#include "polytope/emvp.hpp"
#include "polytope/geo_utils.hpp"

#include <algorithm>
#include <deque>

// 安全走廊生成器
class SFCgenerator{
private:
    double progress, range, eps, minOverlapArea;
    Eigen::Vector3d highCorner, lowCorner;

public:
    SFCgenerator(rclcpp::Node *node){
        progress = node->declare_parameter("SFCgenerator.progress", 0.0);
        range = node->declare_parameter("SFCgenerator.range", 0.0);
        eps = node->declare_parameter("SFCgenerator.eps", 0.0);
        minOverlapArea = node->declare_parameter("SFCgenerator.min_overlap_area", 0.01);

        std::vector<double> mapBound = node->get_parameter("MapBound").as_double_array();
        lowCorner << mapBound[0], mapBound[2], mapBound[4];
        highCorner << mapBound[1], mapBound[3], mapBound[5];
    }

    bool overlap2D(const Eigen::MatrixX4d& lhs, const Eigen::MatrixX4d& rhs, double thresh){
        const double z = 0.5 * (lowCorner(2) + highCorner(2));
        const double tol = std::max(eps, 1e-9);

        std::vector<Eigen::Vector3d> halfplanes;
        halfplanes.reserve(lhs.rows() + rhs.rows());
        for (int i = 0; i < lhs.rows(); ++i) {
            if (std::hypot(lhs(i, 0), lhs(i, 1)) > tol) {
                halfplanes.emplace_back(lhs(i, 0), lhs(i, 1), lhs(i, 2) * z + lhs(i, 3));
            }
        }
        for (int i = 0; i < rhs.rows(); ++i) {
            if (std::hypot(rhs(i, 0), rhs(i, 1)) > tol) {
                halfplanes.emplace_back(rhs(i, 0), rhs(i, 1), rhs(i, 2) * z + rhs(i, 3));
            }
        }
        if (halfplanes.size() < 3) {
            return false;
        }

        std::vector<Eigen::Vector2d> vertices;
        for (size_t i = 0; i < halfplanes.size(); ++i) {
            for (size_t j = i + 1; j < halfplanes.size(); ++j) {
                const double a1 = halfplanes[i].x();
                const double b1 = halfplanes[i].y();
                const double d1 = halfplanes[i].z();
                const double a2 = halfplanes[j].x();
                const double b2 = halfplanes[j].y();
                const double d2 = halfplanes[j].z();
                const double det = a1 * b2 - a2 * b1;
                if (std::abs(det) <= tol) {
                    continue;
                }

                const double rhs1 = -d1;
                const double rhs2 = -d2;
                const Eigen::Vector2d point((rhs1 * b2 - b1 * rhs2) / det,
                                            (a1 * rhs2 - rhs1 * a2) / det);

                bool inside = true;
                for (const Eigen::Vector3d &halfplane : halfplanes) {
                    if (halfplane.x() * point.x() + halfplane.y() * point.y() + halfplane.z() > tol) {
                        inside = false;
                        break;
                    }
                }
                if (!inside) {
                    continue;
                }

                bool duplicate = false;
                for (const Eigen::Vector2d &vertex : vertices) {
                    if ((vertex - point).squaredNorm() <= tol * tol) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) {
                    vertices.emplace_back(point);
                }
            }
        }
        if (vertices.size() < 3) {
            return false;
        }

        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        for (const Eigen::Vector2d &vertex : vertices) {
            centroid += vertex;
        }
        centroid /= static_cast<double>(vertices.size());

        std::sort(vertices.begin(), vertices.end(), [&centroid](const Eigen::Vector2d &a,
                                                                const Eigen::Vector2d &b) {
            return std::atan2(a.y() - centroid.y(), a.x() - centroid.x()) <
                   std::atan2(b.y() - centroid.y(), b.x() - centroid.x());
        });

        double area = 0.0;
        for (size_t i = 0; i < vertices.size(); ++i) {
            const Eigen::Vector2d &a = vertices[i];
            const Eigen::Vector2d &b = vertices[(i + 1) % vertices.size()];
            area += a.x() * b.y() - b.x() * a.y();
        }

        return 0.5 * std::abs(area) >= thresh;
    }

    void getSFC(const std::vector<Eigen::Vector3d> &path, 
                const std::vector<Eigen::Vector3d> &points, 
                std::vector<Eigen::MatrixX4d> &hpolys){
        hpolys.clear();

        // 生成安全走廊
        const int n = path.size();
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;

        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a, b = path[0];
        std::vector<Eigen::Vector3d> valid_pc;
        std::vector<Eigen::Vector3d> bs;
        valid_pc.reserve(points.size());
        for (int i = 1; i < n;) {
            a = b;
            if ((a - path[i]).norm() > progress) {
                b = (path[i] - a).normalized() * progress + a;
            } else {
                b = path[i];
                i++;
            }
            bs.emplace_back(b);

            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));
            // cout << "bd " << endl << bd << endl << endl;
            valid_pc.clear();
            for (const Eigen::Vector3d &p : points) {
                if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0) {
                    valid_pc.emplace_back(p);
                }
            }
            // cout << "valid_pc.size(): " << valid_pc.size() << endl;
            if (valid_pc.size() == 0){
                hpolys.emplace_back(bd);
                continue;
            }
            Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());

            Polytope p;
            if (pc.size() != 0) {
                bool success = emvp::emvp(bd, pc, a, b, p, 0.0, false, 1);

                hp = p.GetPlanes();

                if (hpolys.size() != 0) {
                    const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
                    if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() + ((hpolys.back() * ah).array() > -eps).cast<int>().sum()) {
                        emvp::emvp(bd, pc, a, a, p, 0.0, false, 1);
                        gap = p.GetPlanes();
                        hpolys.emplace_back(gap);
                    }
                }

                hpolys.emplace_back(hp);
            }
        }

        // 简化安全走廊
        std::vector<Eigen::MatrixX4d> htemp = hpolys;
        if (htemp.size() == 1) {
            Eigen::MatrixX4d headPoly = htemp.front();
            htemp.insert(htemp.begin(), headPoly);
        }
        hpolys.clear();
        int M = htemp.size();
        Eigen::MatrixX4d hPoly;
        bool overlap;
        std::deque<int> idices;
        idices.push_front(M - 1);
        for (int i = M - 1; i >= 0; i--) {
            for (int j = 0; j < i; j++) {
                if (j < i - 1) {
                    overlap = overlap2D(htemp[i], htemp[j], minOverlapArea);
                } else {
                    overlap = true;
                }
                if (overlap) {
                    idices.push_front(j);
                    i = j + 1;
                    break;
                }
            }
        }

        if (htemp.size() == 0){
            // std::cout << "out of range " << htemp.size() << std::endl;
            hpolys.clear();
            return;
        }

        for (const int &ele : idices) {
            hpolys.push_back(htemp[ele]);
        }
    }
};
