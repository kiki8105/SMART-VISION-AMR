#include <iostream>
#include <iomanip>
#include <memory>
#include <vector>
#include <set>
#include <cmath>
#include "nav2_util/line_iterator.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
int main(){
 unsigned w,h,n,q;double res,ox,oy;std::cin>>w>>h>>res>>ox>>oy;
 auto map=std::make_shared<nav2_costmap_2d::Costmap2D>(w,h,res,ox,oy,0);
 for(unsigned y=0;y<h;y++)for(unsigned x=0;x<w;x++){int v;std::cin>>v;map->setCost(x,y,static_cast<unsigned char>(v));}
 std::cin>>n;nav2_costmap_2d::Footprint footprint;
 for(unsigned i=0;i<n;i++){geometry_msgs::msg::Point p;std::cin>>p.x>>p.y;footprint.push_back(p);}
 nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>> checker(map);
 std::cin>>q;std::cout<<std::setprecision(14);
 for(unsigned i=0;i<q;i++){double x,y,a;std::cin>>x>>y>>a;std::cout<<i<<" "<<checker.footprintCostAtPose(x,y,a,footprint)<<"\n";
 std::set<std::pair<unsigned,unsigned>> hits;
 for(unsigned j=0;j<n;j++){
  auto p=footprint[j],b=footprint[(j+1)%n];unsigned ax,ay,bx,by;
  if(!map->worldToMap(x+cos(a)*p.x-sin(a)*p.y,y+sin(a)*p.x+cos(a)*p.y,ax,ay))continue;
  if(!map->worldToMap(x+cos(a)*b.x-sin(a)*b.y,y+sin(a)*b.x+cos(a)*b.y,bx,by))continue;
  for(nav2_util::LineIterator line(ax,ay,bx,by);line.isValid();line.advance()){
   unsigned lx=line.getX(),ly=line.getY();if(map->getCost(lx,ly)>=254)hits.insert({lx,ly});}
 }
 for(auto hit:hits)std::cerr<<i<<" "<<hit.first<<" "<<hit.second<<" "<<int(map->getCost(hit.first,hit.second))<<"\n";}
}
