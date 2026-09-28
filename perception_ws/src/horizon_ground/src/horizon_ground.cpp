#include "horizon_ground/horizon_ground.h"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>

namespace horizon_ground
{
namespace
{
constexpr int K = 5;                                           // 거리 링 수
const double RE[K + 1] = {2.9, 4.5, 6.5, 9.0, 12.0, 15.6};     // 링 경계 [m], 2.9 m = 지면이 보이기 시작하는 거리
const int NS[K] = {4, 6, 8, 8, 8};                             // 링별 부채꼴 수
constexpr int MAX_NS = 8;
const double FOV = 41.0 * M_PI / 180.0;
constexpr double ZREF = -0.72;                                 // 평지 기준 지면 높이 (센서 기준)
constexpr double TH_SEED = 0.125, TH_DIST = 0.125, FLAT = 0.003, TH = 0.2;
const double UPRIGHT = std::cos(8.0 * M_PI / 180.0);
constexpr size_t NMIN = 15;

double percentile(std::vector<double> v, double q)
{
    std::sort(v.begin(), v.end());
    const double pos = q / 100.0 * (v.size() - 1);
    const size_t i = static_cast<size_t>(std::floor(pos));
    const double f = pos - i;
    return i + 1 < v.size() ? v[i] * (1 - f) + v[i + 1] * f : v[i];
}

void pca(const std::vector<cv::Point3d> &Q, Eigen::Vector3d &n, double &d, double &lam)
{
    Eigen::Vector3d c(0, 0, 0);
    for (const auto &q : Q) c += Eigen::Vector3d(q.x, q.y, q.z);
    c /= static_cast<double>(Q.size());
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    for (const auto &q : Q)
    {
        const Eigen::Vector3d e(q.x - c[0], q.y - c[1], q.z - c[2]);
        C += e * e.transpose();
    }
    C /= static_cast<double>(Q.size() - 1);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
    n = es.eigenvectors().col(0);
    if (n[2] < 0) n = -n;
    d = -n.dot(c);
    lam = es.eigenvalues()[0];
}

double planeZ(const Plane &p, double x, double y) { return p.a * x + p.b * y + p.c; }

// 한 번의 칸별 평면 추정. seeds 가 있으면 통과한 칸의 씨앗을 모은다(전역 평면 재추정용).
std::vector<char> pass(const std::vector<cv::Point3d> &p, const Plane &prior, double elev,
                       std::vector<cv::Point3d> *seeds)
{
    const size_t N = p.size();
    std::vector<int> ri(N);
    std::vector<std::array<int, K>> si(N);
    std::vector<double> r(N);
    std::vector<std::vector<int>> bins(K * MAX_NS);
    for (size_t i = 0; i < N; ++i)
    {
        r[i] = std::hypot(p[i].x, p[i].y);
        const int k = static_cast<int>(std::upper_bound(RE, RE + K + 1, r[i]) - RE) - 1;
        ri[i] = std::max(-1, std::min(k, K - 1));   // -1: 첫 링 안쪽 (첫 링 평면을 연장)
        const double th = std::atan2(p[i].y, p[i].x);
        for (int kk = 0; kk < K; ++kk)
        {
            const int s = static_cast<int>((th + FOV) / (2 * FOV) * NS[kk]);
            si[i][kk] = std::max(0, std::min(s, NS[kk] - 1));
        }
        if (ri[i] >= 0) bins[ri[i] * MAX_NS + si[i][ri[i]]].push_back(static_cast<int>(i));
    }

    std::vector<Plane> PL(K * MAX_NS);
    std::vector<char> has(K * MAX_NS, 0), own(K * MAX_NS, 0);
    for (int k = 0; k < K; ++k)
        for (int s = 0; s < NS[k]; ++s)
        {
            std::vector<cv::Point3d> Z;
            for (int i : bins[k * MAX_NS + s]) Z.push_back(p[i]);
            if (Z.size() >= NMIN)
            {
                std::vector<cv::Point3d> Z2;
                for (const auto &q : Z)
                    if (std::fabs(q.z - planeZ(prior, q.x, q.y)) < 0.3) Z2.push_back(q);
                Z.swap(Z2);
            }
            if (Z.size() < NMIN) continue;

            std::vector<double> zs;
            for (const auto &q : Z) zs.push_back(q.z);
            std::sort(zs.begin(), zs.end());
            const size_t nl = std::max<size_t>(3, std::min<size_t>(20, Z.size() / 5));
            double lpr = 0;
            for (size_t i = 0; i < nl; ++i) lpr += zs[i];
            lpr /= nl;
            std::vector<cv::Point3d> S;
            for (const auto &q : Z)
                if (q.z < lpr + TH_SEED) S.push_back(q);

            Eigen::Vector3d n;
            double d = 0, lam = 0;
            for (int it = 0; it < 3; ++it)
            {
                if (S.size() < NMIN) break;
                pca(S, n, d, lam);
                std::vector<cv::Point3d> S2;
                for (const auto &q : Z)
                    if (std::fabs(n[0] * q.x + n[1] * q.y + n[2] * q.z + d) < TH_DIST) S2.push_back(q);
                S.swap(S2);
            }
            if (S.size() < NMIN) continue;
            pca(S, n, d, lam);
            std::vector<double> res;
            for (const auto &q : S) res.push_back(n[0] * q.x + n[1] * q.y + n[2] * q.z + d);
            d -= percentile(res, 25);   // 씨앗 띠 때문에 평면이 지면보다 떠 있는 것을 내린다
            const Plane pl{-n[0] / n[2], -n[1] / n[2], -d / n[2]};
            const double rc = (RE[k] + RE[k + 1]) / 2, tc = -FOV + (s + 0.5) * 2 * FOV / NS[k];
            const double xc = rc * std::cos(tc), yc = rc * std::sin(tc);
            const double ed = std::fabs(planeZ(pl, xc, yc) - planeZ(prior, xc, yc));
            if (n[2] >= UPRIGHT && ed <= elev && lam <= FLAT)
            {
                PL[k * MAX_NS + s] = pl;
                has[k * MAX_NS + s] = own[k * MAX_NS + s] = 1;
                if (seeds) seeds->insert(seeds->end(), S.begin(), S.end());
            }
        }

    // 실패 칸: 같은 링 이웃 → 앞뒤 링의 겹치는 칸 평균, 없으면 사전 평면 (앞에서 채운 대체값도 이웃으로 쓴다)
    for (int k = 0; k < K; ++k)
        for (int s = 0; s < NS[k]; ++s)
        {
            if (own[k * MAX_NS + s]) continue;
            Plane m{0, 0, 0};
            int cnt = 0;
            auto add = [&](int kk, int t) {
                if (kk < 0 || kk >= K || t < 0 || t >= NS[kk] || !has[kk * MAX_NS + t]) return;
                const Plane &q = PL[kk * MAX_NS + t];
                m.a += q.a; m.b += q.b; m.c += q.c; ++cnt;
            };
            add(k, s - 1);
            add(k, s + 1);
            for (int kk : {k - 1, k + 1})
                if (kk >= 0 && kk < K) add(kk, static_cast<int>((s + 0.5) * NS[kk] / NS[k]));
            if (cnt) { m.a /= cnt; m.b /= cnt; m.c /= cnt; PL[k * MAX_NS + s] = m; }
            else PL[k * MAX_NS + s] = prior;
            has[k * MAX_NS + s] = 1;
        }

    std::vector<char> ng(N);
    for (size_t i = 0; i < N; ++i)
    {
        Plane pl = prior;
        if (r[i] < RE[K])
        {
            const int k = std::max(ri[i], 0);
            pl = PL[k * MAX_NS + si[i][k]];
        }
        ng[i] = (p[i].z - planeZ(pl, p[i].x, p[i].y)) > TH;
    }
    return ng;
}
}  // namespace

std::vector<char> nonGround(const std::vector<cv::Point3d> &pts, Plane *ground)
{
    std::vector<cv::Point3d> seeds;
    pass(pts, {0, 0, ZREF}, 0.3, &seeds);
    Plane prior{0, 0, ZREF};
    if (seeds.size() >= 50)
    {
        Eigen::MatrixXd A(seeds.size(), 3);
        Eigen::VectorXd z(seeds.size());
        for (size_t i = 0; i < seeds.size(); ++i)
        {
            A(i, 0) = seeds[i].x; A(i, 1) = seeds[i].y; A(i, 2) = 1; z[i] = seeds[i].z;
        }
        const Eigen::Vector3d x = A.colPivHouseholderQr().solve(z);
        if (std::fabs(x[0]) <= 0.1 && std::fabs(x[1]) <= 0.1 && std::fabs(x[2] - ZREF) <= 0.25)
            prior = {x[0], x[1], x[2]};
    }
    if (ground) *ground = prior;
    return pass(pts, prior, 0.15, nullptr);
}
}  // namespace horizon_ground

extern "C" void horizon_ground_non_ground(const double *xyz, int n, unsigned char *mask, double *plane)
{
    std::vector<cv::Point3d> pts(n);
    for (int i = 0; i < n; ++i) pts[i] = cv::Point3d(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]);
    horizon_ground::Plane g;
    const std::vector<char> ng = horizon_ground::nonGround(pts, &g);
    for (int i = 0; i < n; ++i) mask[i] = ng[i];
    plane[0] = g.a; plane[1] = g.b; plane[2] = g.c;
}
