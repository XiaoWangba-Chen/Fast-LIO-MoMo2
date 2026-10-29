#pragma once
#include <algorithm>
#include <cmath>
namespace fastlio {
// Point curvature stores milliseconds relative to scan_start. Rolling sensors
// may overlap adjacent scans; retain the still-new portion without rewinding.
template<class Cloud>
bool clipScanToBoundary(Cloud& cloud,double& scan_start,double scan_end,double committed) {
  if(scan_end<=committed+1e-9)return false;
  if(scan_start>=committed)return !cloud.points.empty();
  const double offset_ms=1000*(committed-scan_start);
  auto& points=cloud.points;
  points.erase(std::remove_if(points.begin(),points.end(),[&](const auto& p){return p.curvature<offset_ms;}),points.end());
  for(auto& p:points)p.curvature=std::max(0.,double(p.curvature)-offset_ms);
  scan_start=committed;
  return !points.empty();
}
}
