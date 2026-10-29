#include "fast_lio/degeneracy.hpp"
#include <Eigen/Eigenvalues>
namespace fastlio {
DegeneracyStatus Degeneracy::analyze(const Eigen::MatrixXd& H,double variance,int candidates,double residual,bool commit) {
  DegeneracyStatus s;
  if(!enabled) {if(commit)status_=s;return s;}
  s.match_ratio=static_cast<double>(H.rows())/std::max(1,candidates); s.residual=residual;
  Mat6 scale=Mat6::Identity(); scale.bottomRightCorner<3,3>()/=rotation_scale;
  if(H.rows()>0) {
    const Eigen::MatrixXd A=H.leftCols(6)*scale;
    s.information=A.transpose()*A/std::max(variance,1e-12);
    Eigen::SelfAdjointEigenSolver<Mat6> solver(s.information/std::max<Eigen::Index>(1,H.rows()));
    s.eigenvalues=solver.eigenvalues();
    double threshold=std::max(absolute_threshold,relative_threshold*s.eigenvalues.maxCoeff());
    for(int i=0;i<6;++i) if(s.eigenvalues[i]<threshold) {
      s.weak_projector+=solver.eigenvectors().col(i)*solver.eigenvectors().col(i).transpose(); ++s.weak_dimensions;
    }
  }
  if(H.rows()<minimum_points||s.match_ratio<minimum_match_ratio) s.reason=DegeneracyStatus::Insufficient;
  else if(!std::isfinite(residual)||residual>maximum_residual) s.reason=DegeneracyStatus::Residual;
  else if(s.weak_dimensions>0) s.reason=DegeneracyStatus::Geometry;
  s.severe=s.reason==DegeneracyStatus::Insufficient||s.reason==DegeneracyStatus::Residual;
  if(commit) {
    if(s.reason!=DegeneracyStatus::Healthy) {++bad_;good_=0;} else {++good_;bad_=0;}
    s.degraded=status_.degraded;
    if(bad_>=enter_frames) s.degraded=true;
    if(good_>=exit_frames) s.degraded=false;
    // Recover map insertion after consecutive valid matches, even if some
    // directions remain geometrically weak. Geometric weakness alone is not a
    // matching failure and must not latch the severe state forever.
    if(s.severe)recoverable_good_=0;else ++recoverable_good_;
    s.severe=s.severe||(status_.severe&&recoverable_good_<exit_frames);
    status_=s;
  } else s.degraded=status_.degraded;
  return s;
}
Eigen::MatrixXd Degeneracy::compensate(const Eigen::MatrixXd& H,const DegeneracyStatus& s) const {
  if(!enabled||s.weak_dimensions==0) return H;
  Mat6 scale=Mat6::Identity(); scale.bottomRightCorner<3,3>()/=rotation_scale;
  Eigen::MatrixXd out=H;
  Eigen::MatrixXd A=H.leftCols(6)*scale;
  Eigen::SelfAdjointEigenSolver<Mat6> solver(A.transpose()*A);
  for(int i=0;i<6;++i){
    auto v=solver.eigenvectors().col(i);
    if(v.dot(s.weak_projector*v)<.5||solver.eigenvalues()[i]<1e-12)continue;
    Eigen::VectorXd u=A*v/std::sqrt(solver.eigenvalues()[i]);
    out-=u*(u.transpose()*H);
  }
  return out;
}
}
