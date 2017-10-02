#include <iostream>
#include <lcm/lcm.h>
#include "pronto_vis/pronto_vis.hpp"
#include "lcmtypes/visualization.h"
//#include "visualization/collections.hpp"
//#include <zlib.h>

#define PCL_VERBOSITY_LEVEL L_ERROR

using namespace std;
using namespace boost::assign; // bring 'operator+=()' into scope

pronto_vis::pronto_vis (lcm_t* publish_lcm):
    publish_lcm_(publish_lcm){

    // Duplicates the list in collections renderer:
    colors+= 
        51/255.0, 160/255.0, 44/255.0,  //0
        166/255.0, 206/255.0, 227/255.0,
        178/255.0, 223/255.0, 138/255.0,//6
        31/255.0, 120/255.0, 180/255.0,
        251/255.0, 154/255.0, 153/255.0,// 12
        227/255.0, 26/255.0, 28/255.0,
        253/255.0, 191/255.0, 111/255.0,// 18
        106/255.0, 61/255.0, 154/255.0,
        255/255.0, 127/255.0, 0/255.0, // 24
        202/255.0, 178/255.0, 214/255.0,
        1.0, 0.0, 0.0, // red // 30
        0.0, 1.0, 0.0, // green
        0.0, 0.0, 1.0, // blue// 36
        1.0, 1.0, 0.0,
        1.0, 0.0, 1.0, // 42
        0.0, 1.0, 1.0,
        0.5, 1.0, 0.0,
        1.0, 0.5, 0.0,
        0.5, 0.0, 1.0,
        1.0, 0.0, 0.5,
        0.0, 0.5, 1.0,
        0.0, 1.0, 0.5,
        1.0, 0.5, 0.5,
        0.5, 1.0, 0.5,
        0.5, 0.5, 1.0,
        0.5, 0.5, 1.0,
        0.5, 1.0, 0.5,
        0.5, 0.5, 1.0;
      
}


void pronto_vis::text_collection_to_lcm(int text_collection_id,
                            int object_collection_id, string text_collection_name,
                            std::vector<std::string >& labels, 
                            std::vector<int64_t>& object_ids) {

  vs_text_collection_t tcolls;
  tcolls.id = text_collection_id;
  tcolls.name =  (char*) text_collection_name.c_str();
  tcolls.type = 0;
  tcolls.reset = true; // true will delete them from the viewer
  tcolls.n= labels.size();
  vs_text_t texts[tcolls.n];
  for (int i=0;i< tcolls.n;i++){
    texts[i].id = (int64_t) object_ids[i]; //tiles[i].utime  ; doesnt give correct utime for some
    texts[i].collection_id = object_collection_id;
    texts[i].object_id = object_ids[i];//tiles[i].utime; doesnt give correct utime for some
    texts[i].text= (char*) labels[i].c_str();   
  }
  tcolls.texts = texts;
  vs_text_collection_t_publish(publish_lcm_, "TEXT_COLLECTION", &tcolls);  
}


void pronto_vis::link_to_lcm(link_cfg lcfg, link_data ldata){

  vs_link_collection_t lcolls;
  lcolls.id = lcfg.id;
  lcolls.name =  (char*) lcfg.name.c_str();
  lcolls.type = lcfg.type;
  lcolls.reset = lcfg.reset; // true will delete them from the viewer
  lcolls.nlinks= 1;
  
  vs_link_t links[lcolls.nlinks];
  links[0].id = (int64_t) ldata.id;
  links[0].collection1 = (int32_t) ldata.collection1;
  links[0].id1 = (int64_t) ldata.id1;
  links[0].collection2 = (int32_t) ldata.collection2;
  links[0].id2 = (int64_t) ldata.id2;
  lcolls.links = links;
  vs_link_collection_t_publish(publish_lcm_, "LINK_COLLECTION", &lcolls);  
}


void pronto_vis::link_collection_to_lcm(link_cfg lcfg, std::vector<link_data> ldata){

  vs_link_collection_t lcolls;
  lcolls.id = lcfg.id;
  lcolls.name =  (char*) lcfg.name.c_str();
  lcolls.type = lcfg.type;
  lcolls.reset = lcfg.reset; // true will delete them from the viewer
  lcolls.nlinks= ldata.size();
  vs_link_t links[lcolls.nlinks];
  for (int i=0;i< lcolls.nlinks;i++){
    links[i].id = (int64_t) ldata[i].id;
    links[i].collection1 = (int32_t) ldata[i].collection1;
    links[i].id1 = (int64_t) ldata[i].id1;
    links[i].collection2 = (int32_t) ldata[i].collection2;
    links[i].id2 = (int64_t) ldata[i].id2;
  }
  lcolls.links = links;
  vs_link_collection_t_publish(publish_lcm_, "LINK_COLLECTION", &lcolls);  
}


void pronto_vis::pose_collection_to_lcm_from_list(int id, std::vector<Isometry3dTime> & posesT){
 for (size_t i=0; i < obj_cfg_list.size() ; i++){
   if (id == obj_cfg_list[i].id ){
     pose_collection_to_lcm(obj_cfg_list[i],posesT);
       return;
   }
 }
}


void pronto_vis::pose_collection_to_lcm(obj_cfg ocfg, std::vector<Isometry3dTime> & posesT){
  // Send a pose
  vs_object_collection_t objs;
  objs.id = ocfg.id;
  objs.name = (char*)  ocfg.name.c_str();
  objs.type = ocfg.type;
  objs.reset = ocfg.reset; // true will delete them from the viewer
  objs.nobjects = posesT.size();
  vs_object_t poses[objs.nobjects];
  for (int i=0;i< objs.nobjects;i++){
    poses[i].id = (int64_t) posesT[i].utime;// which specific pose
    poses[i].x = posesT[i].pose.translation().x();
    poses[i].y = posesT[i].pose.translation().y();
    poses[i].z = posesT[i].pose.translation().z();
    Eigen::Quaterniond r(posesT[i].pose.rotation());
    poses[i].qw = r.w();
    poses[i].qx = r.x();
    poses[i].qy = r.y();
    poses[i].qz = r.z();
  }
  objs.objects = poses;
  vs_object_collection_t_publish(publish_lcm_, "OBJECT_COLLECTION", &objs);
}


void pronto_vis::pose_to_lcm_from_list(int id,Isometry3dTime& poseT){
 for (size_t i=0; i < obj_cfg_list.size() ; i++){
   if (id == obj_cfg_list[i].id ){
     pose_to_lcm(obj_cfg_list[i],poseT);
       return;
   }
 }
}


void pronto_vis::pose_to_lcm(obj_cfg ocfg, Isometry3dTime& poseT){
  // Send a pose
  vs_object_collection_t objs;
  objs.id = ocfg.id;
  objs.name = (char*)  ocfg.name.c_str();
  objs.type = ocfg.type;
  objs.reset = ocfg.reset; // true will delete them from the viewer
  objs.nobjects = 1;
  vs_object_t poses[objs.nobjects];
  poses[0].id = (int64_t) poseT.utime;// which specific pose
  poses[0].x = poseT.pose.translation().x();
  poses[0].y = poseT.pose.translation().y();
  poses[0].z = poseT.pose.translation().z();
  Eigen::Quaterniond r(poseT.pose.rotation());
  poses[0].qw = r.w();
  poses[0].qx = r.x();
  poses[0].qy = r.y();
  poses[0].qz = r.z();
  objs.objects = poses;
  vs_object_collection_t_publish(publish_lcm_, "OBJECT_COLLECTION", &objs);
}


void pronto_vis::pose_collection_reset(int id, std::string name){
  vs_object_collection_t objs;
  objs.id = id;
  objs.name =(char*) name.c_str();
  objs.type =5;
  objs.reset = true;
  objs.nobjects = 0;
  vs_object_collection_t_publish(publish_lcm_, "OBJECT_COLLECTION", &objs);
}


//
void pronto_vis::ptcld_collection_to_lcm(ptcld_cfg pcfg, std::vector< pronto::PointCloud > &clouds,
            int64_t obj_id, int64_t ptcld_id){
  std::cout << "This function is depreciated, please replace with method providing lists of ids\n";

  vs_point3d_list_collection_t plist_coll;
  plist_coll.id = pcfg.id;
  plist_coll.name =(char*)   pcfg.name.c_str();
  plist_coll.type =pcfg.type; // collection of points
  plist_coll.reset = pcfg.reset;
  plist_coll.nlists = clouds.size(); // number of seperate sets of points
  vs_point3d_list_t plist[plist_coll.nlists];

  for (size_t i=0; i < clouds.size() ; i++){  // loop here for many lists
    size_t npts = clouds[i].points.size();

    vs_point3d_list_t* this_plist = &(plist[i]);
    // 3.0: header
    this_plist->id = ptcld_id+i; // which specific cloud is this     ptcoll_cfg.point_lists_id;
    this_plist->collection = pcfg.obj_coll;
    this_plist->element_id = obj_id; // which specific pose axis typically a timestamp
    // 3.1: points/entries (rename)
    vs_point3d_t* points = new vs_point3d_t[npts];
    this_plist->npoints = npts;
    // 3.2: colors:
    vs_color_t* colors = new vs_color_t[npts];
    // TODO: this sends the points, but they are not used. Should avoid sending altogether
    if (pcfg.use_rgb == -1)
      this_plist->ncolors = 0; // have viewer pick the color
    else
      this_plist->ncolors = npts;
    // 3.3: normals:
    this_plist->nnormals = 0;
    this_plist->normals = NULL;
    // 3.4: point ids:
    this_plist->npointids = 0;//cloud.points.size();
    int64_t* pointsids= NULL;//new int64_t[ cloud.points.size() ];

    float rgba[4];
    for(size_t j=0; j<npts; j++) {  //Nransac
      if (  pcfg.use_rgb == 1){// use the rgb value
        rgba[0] = pcfg.rgb[0];
        rgba[1] = pcfg.rgb[1];
        rgba[2] = pcfg.rgb[2];
      }else{
        rgba[0] = clouds[i].points[j].r/255.0;
        rgba[1] = clouds[i].points[j].g/255.0;
        rgba[2] = clouds[i].points[j].b/255.0;
      }

      colors[j].r = rgba[0]; // points_collection values range 0-1
      colors[j].g = rgba[1];
      colors[j].b = rgba[2];
      points[j].x = clouds[i].points[j].x;
      points[j].y = clouds[i].points[j].y;
      points[j].z = clouds[i].points[j].z;
    }
    this_plist->colors = colors;
    this_plist->points = points;
    this_plist->pointids = pointsids;
  }
  
  plist_coll.point_lists = plist;
  vs_point3d_list_collection_t_publish(publish_lcm_,"POINTS_COLLECTION",&plist_coll);

  for (int i=0;i<plist_coll.nlists;i++) {
      delete [] plist_coll.point_lists[i].points;
      delete [] plist_coll.point_lists[i].colors;
  }     
}



//
void pronto_vis::ptcld_collection_to_lcm(ptcld_cfg pcfg, std::vector< pronto::PointCloud > &clouds,
            std::vector<int64_t> &obj_ids, std::vector<int64_t> &ptcld_ids){

  vs_point3d_list_collection_t plist_coll;
  plist_coll.id = pcfg.id;
  plist_coll.name =(char*)   pcfg.name.c_str();
  plist_coll.type =pcfg.type; // collection of points
  plist_coll.reset = pcfg.reset;
  plist_coll.nlists = clouds.size(); // number of seperate sets of points
  vs_point3d_list_t plist[plist_coll.nlists];

  for (size_t i=0; i < clouds.size() ; i++){  // loop here for many lists
    size_t npts = clouds[i].points.size();

    vs_point3d_list_t* this_plist = &(plist[i]);
    // 3.0: header
    this_plist->id = ptcld_ids[i]; // which specific cloud is this     ptcoll_cfg.point_lists_id;
    this_plist->collection = pcfg.obj_coll;
    this_plist->element_id = obj_ids[i]; // which specific pose axis typically a timestamp
    // 3.1: points/entries (rename)
    vs_point3d_t* points = new vs_point3d_t[npts];
    this_plist->npoints = npts;
    // 3.2: colors:
    vs_color_t* colors = new vs_color_t[npts];
    // TODO: this sends the points, but they are not used. Should avoid sending altogether
    if (pcfg.use_rgb == -1)
      this_plist->ncolors = 0; // have viewer pick the color
    else
      this_plist->ncolors = npts;
    // 3.3: normals:
    this_plist->nnormals = 0;
    this_plist->normals = NULL;
    // 3.4: point ids:
    this_plist->npointids = 0;//cloud.points.size();
    int64_t* pointsids= NULL;//new int64_t[ cloud.points.size() ];

    float rgba[4];
    for(size_t j=0; j<npts; j++) {  //Nransac
      if (  pcfg.use_rgb == 1){// use the rgb value
        rgba[0] = pcfg.rgb[0];
        rgba[1] = pcfg.rgb[1];
        rgba[2] = pcfg.rgb[2];
      }else{
        rgba[0] = clouds[i].points[j].r/255.0;
        rgba[1] = clouds[i].points[j].g/255.0;
        rgba[2] = clouds[i].points[j].b/255.0;
      }

      colors[j].r = rgba[0]; // points_collection values range 0-1
      colors[j].g = rgba[1];
      colors[j].b = rgba[2];
      points[j].x = clouds[i].points[j].x;
      points[j].y = clouds[i].points[j].y;
      points[j].z = clouds[i].points[j].z;
    }
    this_plist->colors = colors;
    this_plist->points = points;
    this_plist->pointids = pointsids;
  }
  
  plist_coll.point_lists = plist;
  vs_point3d_list_collection_t_publish(publish_lcm_,"POINTS_COLLECTION",&plist_coll);

  for (int i=0;i<plist_coll.nlists;i++) {
      delete [] plist_coll.point_lists[i].points;
      delete [] plist_coll.point_lists[i].colors;
  }     
}



void pronto_vis::ptcld_to_lcm_from_list(int id, pronto::PointCloud &cloud, int64_t obj_id, int64_t ptcld_id){
 for (size_t i=0; i < ptcld_cfg_list.size() ; i++){
   if (id == ptcld_cfg_list[i].id ){
     ptcld_to_lcm(ptcld_cfg_list[i],cloud,obj_id,ptcld_id);
       return;
   }
 }
}

void pronto_vis::ptcld_to_lcm(ptcld_cfg pcfg, pronto::PointCloud &cloud, int64_t obj_id, int64_t ptcld_id){
  std::vector<Eigen::Vector3d> empty_normal_list;
  ptcld_to_lcm(pcfg, cloud, empty_normal_list, obj_id, ptcld_id);
}


void pronto_vis::ptcld_to_lcm(ptcld_cfg pcfg, pronto::PointCloud &cloud,
    std::vector< Eigen::Vector3d> &cloud_normals,
    int64_t obj_id, int64_t ptcld_id){

  int npts = cloud.points.size();

  vs_point3d_list_collection_t plist_coll;
  plist_coll.id = pcfg.id;
  plist_coll.name =(char*)   pcfg.name.c_str();
  plist_coll.type =pcfg.type; // collection of points
  plist_coll.reset = pcfg.reset;
  plist_coll.nlists = 1; // number of seperate sets of points
  vs_point3d_list_t plist[plist_coll.nlists];

  // loop here for many lists
  vs_point3d_list_t* this_plist = &(plist[0]);
  // 3.0: header
  this_plist->id = ptcld_id; // which specific cloud is this     ptcoll_cfg.point_lists_id;
  this_plist->collection = pcfg.obj_coll;
  this_plist->element_id = obj_id; // which specific pose axis typically a timestamp
  // 3.1: points/entries (rename)
  vs_point3d_t* points = new vs_point3d_t[npts];
  this_plist->npoints = npts;
  // 3.2: colors:
  vs_color_t* colors = new vs_color_t[npts];

  // TODO: this sends the points, but they are not used. Should avoid sending altogether
  if ( pcfg.use_rgb == -1 )
    this_plist->ncolors = 0; // have the viewer choose the colors.
  else
    this_plist->ncolors = npts;

  // 3.3: normals:
  vs_point3d_t* normals = new vs_point3d_t[npts];
  if (cloud_normals.size() == npts)
    this_plist->nnormals = npts;
  else
    this_plist->nnormals = 0;

  // 3.4: point ids:
  this_plist->npointids = 0;//cloud.points.size();
  int64_t* pointsids= NULL;//new int64_t[ cloud.points.size() ];

  float rgba[4];
  for(int j=0; j<npts; j++) {  //Nransac
    if (  pcfg.use_rgb == 1){// use the rgb value
      //rgba[3] = ptcoll_cfg.rgba[3];
      rgba[0] = pcfg.rgb[0];
      rgba[1] = pcfg.rgb[1];
      rgba[2] = pcfg.rgb[2];
    }else{
      rgba[0] = cloud.points[j].r/255.0;
      rgba[1] = cloud.points[j].g/255.0;
      rgba[2] = cloud.points[j].b/255.0;
    }

    colors[j].r = rgba[0]; // points_collection values range 0-1
    colors[j].g = rgba[1];
    colors[j].b = rgba[2];
    points[j].x = cloud.points[j].x;
    points[j].y = cloud.points[j].y;
    points[j].z = cloud.points[j].z;

    if (cloud_normals.size() == npts){
      normals[j].x = cloud_normals[j][0];
      normals[j].y = cloud_normals[j][1];
      normals[j].z = cloud_normals[j][2];
    }

  }

  this_plist->colors = colors;
  this_plist->points = points;
  this_plist->normals = normals;
  this_plist->pointids = pointsids;
  plist_coll.point_lists = plist;
  vs_point3d_list_collection_t_publish(publish_lcm_,"POINTS_COLLECTION",&plist_coll);


  delete pointsids;
  delete colors;
  delete points;
}


void pronto_vis::ptcld_collection_reset(int id, std::string name){
  vs_point3d_list_collection_t point_lists;
  point_lists.id = id;
  point_lists.name =(char*) name.c_str();
  point_lists.type =5;
  point_lists.reset = true;
  point_lists.nlists= 0;
  vs_point3d_list_collection_t_publish(publish_lcm_, "POINTS_COLLECTION", &point_lists);
}


void pronto_vis::transformPointCloud(pronto::PointCloud &cloud_in, pronto::PointCloud &cloud_out, Eigen::Affine3f transform){
  int npts = cloud_in.points.size();
  cloud_out.points.resize(npts);
  for(int j=0; j<npts; j++) {
    Eigen::Vector3f pt_out = transform* Eigen::Vector3f(cloud_in.points[j].x, cloud_in.points[j].y, cloud_in.points[j].z);
    cloud_out.points[j].x = pt_out(0);
    cloud_out.points[j].y = pt_out(1);
    cloud_out.points[j].z = pt_out(2);
    cloud_out.points[j].r = cloud_in.points[j].r;
    cloud_out.points[j].g = cloud_in.points[j].g;
    cloud_out.points[j].b = cloud_in.points[j].b;
  }
}


bool pronto_vis::interpolateScan(const std::vector<float>& iRanges,
                const double iTheta0, const double iThetaStep,
                const Eigen::Isometry3d& iPose0,
                const Eigen::Isometry3d& iPose1,
                std::vector<Eigen::Vector3f>& oPoints) {
  const int n = iRanges.size();
  if (n < 2) return false;
  const double tStep = 1.0/(n-1);
  Eigen::Quaterniond q0(iPose0.linear());
  Eigen::Quaterniond q1(iPose1.linear());
  Eigen::Vector3d pos0(iPose0.translation());
  Eigen::Vector3d pos1(iPose1.translation());
  oPoints.resize(n);
  double t = 0;
  double theta = iTheta0;
  for (int i = 0; i < n; ++i, t += tStep, theta += iThetaStep) {
    Eigen::Quaterniond q = q0.slerp(t,q1);
    Eigen::Vector3d pos = (1-t)*pos0 + t*pos1;
    Eigen::Vector3d pt = iRanges[i]*Eigen::Vector3d(cos(theta), sin(theta), 0);
    oPoints[i] = (q*pt + pos).cast<float>();
  }
  return true;
}


void pronto_vis::convertLidar(std::vector< float > ranges, int numPoints, double thetaStart,
        double thetaStep,
        pronto::PointCloud* &cloud,
	double minRange, double maxRange,
	double validRangeStart, double validRangeEnd){
  int count = 0;
  double theta = thetaStart;

  cloud->points.resize (numPoints);

  // minRange was 0.1 until march 2013
  for (int i = 0; i < numPoints; i++) {
      if (ranges[i] > minRange && ranges[i] < maxRange && theta > validRangeStart
              && theta < validRangeEnd) { 
          //hokuyo driver seems to report maxRanges as .001 :-/
          //project to body centered coordinates
          cloud->points[count].x = ranges[i] * cos(theta);
          cloud->points[count].y = ranges[i] * sin(theta);
          cloud->points[count].r = 0; cloud->points[count].g = 0; cloud->points[count].b = 0;
          count++;
      }
      theta += thetaStep;
  }
  // Resize outgoing cloud
  cloud->points.resize (count);
}


void pronto_vis::writePCD(std::string filename, pronto::PointCloud &cloud){

  std::ofstream of (filename.c_str() );
  if (of.is_open())
  {
    of << "# .PCD v.7 - Point Cloud Data file format\n";
    of << "VERSION .7\n";
    of << "FIELDS x y z rgb\n";
    of << "SIZE 4 4 4 4\n";
    of << "TYPE F F F F\n";
    of << "COUNT 1 1 1 1\n";
    of << "WIDTH "<< cloud.points.size() << "\n";
    of << "HEIGHT 1\n";
    of << "VIEWPOINT 0 0 0 1 0 0 0\n";
    of << "POINTS "<< cloud.points.size() << "\n";
    of << "DATA ascii\n";
    for(size_t i=0; i < cloud.points.size() ; i++){
      of << cloud.points[i].x << " "
         << cloud.points[i].y << " "
         << cloud.points[i].z << " "
         << 0 << "\n";
    }
    of.close();
  }
}

//#ifdef USE_PRONTO_VIS_PCL
//// PCL::PointCloud publishes and conversions (from here to end)
void pronto_vis::convertCloudPclToPronto(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pronto::PointCloud &cloud_out){
  int npts = cloud.points.size();
  cloud_out.points.resize(npts);
  for(int j=0; j<npts; j++) {
    cloud_out.points[j].x = cloud.points[j].x;
    cloud_out.points[j].y = cloud.points[j].y;
    cloud_out.points[j].z = cloud.points[j].z;
    cloud_out.points[j].r = cloud.points[j].r;
    cloud_out.points[j].g = cloud.points[j].g;
    cloud_out.points[j].b = cloud.points[j].b;
  }
}

void pronto_vis::convertCloudProntoToPcl(pronto::PointCloud &cloud, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out){
  int npts = cloud.points.size();
  cloud_out.points.resize(npts);
  for(int j=0; j<npts; j++) {
    cloud_out.points[j].x = cloud.points[j].x;
    cloud_out.points[j].y = cloud.points[j].y;
    cloud_out.points[j].z = cloud.points[j].z;
    cloud_out.points[j].r = cloud.points[j].r;
    cloud_out.points[j].g = cloud.points[j].g;
    cloud_out.points[j].b = cloud.points[j].b;
  }
}

void pronto_vis::ptcld_collection_to_lcm_from_list(int id, std::vector< pcl::PointCloud<pcl::PointXYZRGB> > &clouds,
  int64_t obj_id, int64_t ptcld_id){
 for (size_t i=0; i < ptcld_cfg_list.size() ; i++){
   if (id == ptcld_cfg_list[i].id ){
     ptcld_collection_to_lcm(ptcld_cfg_list[i],clouds, obj_id, ptcld_id);
       return;
   }
 }
}


void pronto_vis::ptcld_to_lcm_from_list(int id, pcl::PointCloud<pcl::PointXYZRGB> &cloud, int64_t obj_id, int64_t ptcld_id){
 for (size_t i=0; i < ptcld_cfg_list.size() ; i++){
   if (id == ptcld_cfg_list[i].id ){
     ptcld_to_lcm(ptcld_cfg_list[i],cloud,obj_id,ptcld_id);
       return;
   }
 }
}

void pronto_vis::ptcld_to_lcm(ptcld_cfg pcfg, pcl::PointCloud<pcl::PointXYZRGB> &cloud, int64_t obj_id, int64_t ptcld_id){
  pronto::PointCloud* cloud_out (new pronto::PointCloud);
  convertCloudPclToPronto(cloud,*cloud_out);
  ptcld_to_lcm(pcfg, *cloud_out, obj_id, ptcld_id);  
}


void pronto_vis::ptcld_to_lcm(ptcld_cfg pcfg, pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
    std::vector< Eigen::Vector3d> &cloud_normals,
    int64_t obj_id, int64_t ptcld_id){
  pronto::PointCloud* cloud_out (new pronto::PointCloud);
  convertCloudPclToPronto(cloud,*cloud_out);
  ptcld_to_lcm(pcfg, *cloud_out, cloud_normals, obj_id, ptcld_id);  
}

void pronto_vis::ptcld_collection_to_lcm(ptcld_cfg pcfg, std::vector< pcl::PointCloud<pcl::PointXYZRGB> > &clouds,
        std::vector<int64_t> &obj_ids, std::vector<int64_t> &ptcld_ids){

  std::vector < pronto::PointCloud > clouds_out;
  for (size_t i=0; i < clouds.size() ; i++){
    pronto::PointCloud* cloud_out (new pronto::PointCloud);
    convertCloudPclToPronto(clouds[i],*cloud_out);
    clouds_out.push_back(*cloud_out);
  }
  ptcld_collection_to_lcm(pcfg, clouds_out, obj_ids, ptcld_ids);
}

void pronto_vis::ptcld_collection_to_lcm(ptcld_cfg pcfg, std::vector< pcl::PointCloud<pcl::PointXYZRGB> > &clouds,
            int64_t obj_id, int64_t ptcld_id){
  std::vector < pronto::PointCloud > clouds_out;
  for (size_t i=0; i < clouds.size() ; i++){
    pronto::PointCloud* cloud_out (new pronto::PointCloud);
    convertCloudPclToPronto(clouds[i],*cloud_out);
    clouds_out.push_back(*cloud_out);
  }
  ptcld_collection_to_lcm(pcfg, clouds, obj_id, ptcld_id);
}

void pronto_vis::mesh_to_lcm_from_list(int id, pcl::PolygonMesh::Ptr mesh,
    int64_t obj_id, int64_t ptcld_id,
    bool sendSubset, const vector<int> &SubsetIndicies){
  for (size_t i=0; i < ptcld_cfg_list.size() ; i++){
    if (id == ptcld_cfg_list[i].id ){
      mesh_to_lcm(ptcld_cfg_list[i],mesh,obj_id,ptcld_id, sendSubset, SubsetIndicies);
       return;
    }
  }
}


void pronto_vis::mesh_to_lcm(ptcld_cfg pcfg, pcl::PolygonMesh::Ptr mesh,
      int64_t obj_id, int64_t ptcld_id,
      bool sendSubset, const vector<int> &SubsetIndicies){

  // skip_above_z doesnt display the z dimension
  //TODO: have general method for skipping outside x,y,z +/-
  int N_polygons;
  if (!sendSubset){
    N_polygons = mesh->polygons.size ();
  }else{
    N_polygons = SubsetIndicies.size ();
  }

  vs_point3d_list_collection_t point_lists;
  point_lists.id = pcfg.id;
  point_lists.name = (char *)pcfg.name.c_str(); // Use channel name?
  point_lists.type = pcfg.type; // collection of POINTS

  point_lists.reset = pcfg.reset;
  point_lists.nlists = N_polygons; // number of seperate sets of points
  vs_point3d_list_t point_list[N_polygons];

  pcl::PointCloud<pcl::PointXYZRGB> newcloud;
  pcl::fromPCLPointCloud2(mesh->cloud, newcloud);
  Eigen::Vector4f tmp;
  for(size_t i=0; i< N_polygons; i++){ // each triangle/polygon
    size_t k;
    if (!sendSubset){ // send all of the mesh
      k = i;
    }else{ // only send some of it:
      k =  SubsetIndicies[i];
    }

    pcl::Vertices apoly_in = mesh->polygons[k];//[i];
    int N_points = apoly_in.vertices.size ();

    vs_point3d_list_t* points = &(point_list[i]); //[i]);
    points->nnormals = 0;
    points->normals = NULL;
    points->npointids = 0;
    points->pointids = NULL;

    vs_color_t* colors = new vs_color_t[N_points];
    // TODO: this sends the points, but they are not used. Should avoid sending altogether
    if (  pcfg.use_rgb == -1)
      points->ncolors = 0;
    else
      points->ncolors = N_points;

    vs_point3d_t* entries = new vs_point3d_t[N_points];
    points->npoints = N_points;

    points->id = i; // ... still i - not k
    points->collection = pcfg.obj_coll;//PoseCollID;//collection.objectCollectionId();
    points->element_id = obj_id;//ptcoll_cfg.element_id;
    float rgba[4];
    for(size_t j=0; j< N_points; j++){ // each point
      uint32_t pt = apoly_in.vertices[j];
      tmp = newcloud.points[pt].getVector4fMap();
      entries[j].x =(float) tmp(0);
      entries[j].y =(float) tmp(1);
      entries[j].z =(float) tmp(2);
      // r,g,b: input is ints 0->255, opengl wants floats 0->1
      if (  pcfg.use_rgb == 1){// use the rgb value
        rgba[0] = pcfg.rgb[0];
        rgba[1] = pcfg.rgb[1];
        rgba[2] = pcfg.rgb[2];
      }else{
        rgba[0] = newcloud.points[pt].r/255.0;
        rgba[1] = newcloud.points[pt].g/255.0;
        rgba[2] = newcloud.points[pt].b/255.0;
      }
      colors[j].r = rgba[0]; // points_collection values range 0-1
      colors[j].g = rgba[1];
      colors[j].b = rgba[2];
    }
    points->points = entries;
    points->colors = colors;
  }
  point_lists.point_lists = point_list;
  vs_point3d_list_collection_t_publish(publish_lcm_,"POINTS_COLLECTION",&point_lists);

  //TODO I don't think i am doing memory management properly!!!
  //  delete colors;
  for (int i=0;i<point_lists.nlists;i++) {
    delete [] point_lists.point_lists[i].points;
    delete [] point_lists.point_lists[i].colors;
  }
}


bool pronto_vis::mergePolygonMesh(pcl::PolygonMesh::Ptr &meshA, pcl::PolygonMesh::Ptr meshB){
  pcl::PointCloud<pcl::PointXYZRGB> cloudA;  
  // HACKY BUG FIX: 
  // issue: if meshA->cloud contains no data, then it contains no cloud.fields
  //        so it will complain and give a warning when we try to copy to cloudA
  //        Failed to find match for field 'x'.
  //        Failed to find match for field 'y'.
  //        Failed to find match for field 'z'.
  //        Failed to find match for field 'rgb'.  
  // Instead dont try to copy if empty...
  if ( meshA->cloud.fields.size()  !=0){
    pcl::fromPCLPointCloud2(meshA->cloud, cloudA);
  }
  int original_size = cloudA.points.size() ;

  //cout << original_size << " is the cloud before (insize) size\n";
  //cout <<  meshA->polygons.size () << "polygons before\n";
  
  int N_polygonsB = meshB->polygons.size ();
  pcl::PointCloud<pcl::PointXYZRGB> cloudB;  
  pcl::fromPCLPointCloud2(meshB->cloud, cloudB);
  Eigen::Vector4f tmp;
  for(size_t i=0; i< N_polygonsB; i++){ // each triangle/polygon
    pcl::Vertices apoly_in = meshB->polygons[i];//[i];
    int N_points = apoly_in.vertices.size ();
    for(size_t j=0; j< N_points; j++){ // each point
      // increment the vertex numbers by the size of the original clouds
      apoly_in.vertices[j] += original_size; 
    }
    meshA->polygons.push_back(apoly_in);
  } 
  cloudA += cloudB;
  pcl::toPCLPointCloud2 (cloudA, meshA->cloud);
  //cout <<  meshA->polygons.size () << "polygons after\n";
  //cout << cloudA.points.size() << " is the cloud inside size\n";
  return true;
}


void pronto_vis::ptcldToOctomapLogFile(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
            std::string filename){
  int npts = cloud.points.size();
  
  std::ofstream outputfile;
  outputfile.open ( filename.c_str() );
  outputfile << "NODE 0 0 0 0 0 0\n";
  for(int j=0; j<npts; j++) {  //Nransac
    outputfile << cloud.points[j].x << " " << cloud.points[j].y << " " << cloud.points[j].z << "\n";
  }    
  outputfile.close();  
  
}


void pronto_vis::savePLYFile(pcl::PolygonMesh::Ptr model,string fname){
  pcl::PointCloud<pcl::PointXYZRGB> bigcloud;  
  pcl::fromPCLPointCloud2(model->cloud, bigcloud);
  Eigen::Vector4f tmp;
  
  FILE * fid;
  fid = fopen (fname.c_str(),"w");

  fprintf (fid, "ply\n");
  fprintf (fid, "format ascii 1.0\n");
  fprintf (fid, "comment native PCL export\n");
  fprintf(fid,"element vertex %d\n", (int) bigcloud.points.size());
  fprintf(fid,"property float x\n");
  fprintf(fid,"property float y\n");
  fprintf(fid,"property float z\n");
  fprintf(fid,"property uchar red\n");
  fprintf(fid,"property uchar green\n");
  fprintf(fid,"property uchar blue\n");
  fprintf(fid,"property uchar alpha\n");

  fprintf(fid,"element face %d\n", (int) model->polygons.size());
  fprintf(fid,"property list uchar int vertex_indices\n");
  fprintf(fid,"end_header\n");
   
  for(size_t i=0; i<bigcloud.size() ; i++){ // each triangle/polygon
    tmp = bigcloud.points[i].getVector4fMap();
    
    stringstream ss;
    ss << tmp(0) << " "
       << tmp(1) << " "
       << tmp(2) << " "
       << (int) bigcloud.points[i].r<< " "
       << (int)  bigcloud.points[i].g << " "
       << (int)  bigcloud.points[i].b << " 255";
      
    fprintf(fid,"%s\n", ss.str().c_str());
  }     
  
  for(size_t i=0; i< model->polygons.size (); i++){ // each triangle/polygon
    pcl::Vertices apoly_in = model->polygons[i];
    stringstream ss;
    int nvert= apoly_in.vertices.size ();
    ss << nvert ;
    for(size_t j=0; j< apoly_in.vertices.size (); j++){ // each point 
      ss << " " << apoly_in.vertices[j] ;
    }
    //cout << ss.str() << "\n";
    fprintf(fid,"%s\n", ss.str().c_str());
  }
  fclose(fid);
}


void pronto_vis::removeColoredPolygons(pcl::PolygonMesh::Ptr meshin_ptr,vector<int> &color ){
  int N_polygons = meshin_ptr->polygons.size ();
  pcl::PointCloud<pcl::PointXYZRGB> bigcloud;  
  pcl::fromPCLPointCloud2(meshin_ptr->cloud, bigcloud);
  
  vector<int> polygon_indices;
  
  for(size_t i=0; i< N_polygons; i++){ // each triangle/polygon
    pcl::Vertices apoly_in = meshin_ptr->polygons[i];
      
    size_t j=0;//for the 1st point in the polygon
    uint32_t pt = apoly_in.vertices[j];
    int in_mesh = (int)i; // inside 3D box

    vector <float> this_color;
    this_color.push_back( bigcloud.points[pt].r);
    this_color.push_back( bigcloud.points[pt].g);
    this_color.push_back( bigcloud.points[pt].b);
    bool match=false;
    if (color[0] == this_color[0]){
      if (color[1] == this_color[1]){
        if (color[2] == this_color[2]){
          match=true;
        }
      }
    }
    if(!match){
      polygon_indices.push_back(in_mesh);
    }
  }
  
  vector <pcl::Vertices> new_verts;
  for(size_t i=0; i< polygon_indices.size(); i++){ // each triangle/polygon
    pcl::Vertices apoly_in = meshin_ptr->polygons[polygon_indices[i]];
    new_verts.push_back(apoly_in);
  }
  meshin_ptr->polygons.clear();
  meshin_ptr->polygons = new_verts;
}


void pronto_vis::getMeshInBoxIndices(pcl::PolygonMesh::Ptr meshin_ptr,
		   vector<double> &center, vector<double> &dgrid,
		   vector<int> &polygon_in_box_indices){
  int N_polygons = meshin_ptr->polygons.size ();
  pcl::PointCloud<pcl::PointXYZRGB> bigcloud;  
  pcl::fromPCLPointCloud2(meshin_ptr->cloud, bigcloud);
  Eigen::Vector4f tmp;
  for(size_t i=0; i< N_polygons; i++){ // each triangle/polygon
    pcl::Vertices apoly_in = meshin_ptr->polygons[i];
    int N_points = apoly_in.vertices.size ();
    for(size_t j=0; j< N_points; j++){ // each point
      uint32_t pt = apoly_in.vertices[j];
      tmp = bigcloud.points[pt].getVector4fMap(); // xyz 
      // are any points of the polygon in the box, if so add them to the new model
      if ((tmp(0) > center[0] - dgrid[0])&& (tmp(0) < center[0] + dgrid[0])) // widtin x
	if ((tmp(1) > center[1] - dgrid[1])&& (tmp(1) < center[1] + dgrid[1])) // within y
	  if ((tmp(2) > center[2] - dgrid[2])&& (tmp(2) < center[2] + dgrid[2])){ // within z
	     int in_mesh = (int)i; // inside 3D box
	     polygon_in_box_indices.push_back(in_mesh);
	     break;
	  }
    }
  }
  cout << polygon_in_box_indices.size() << " is the polygon_in_box_indices size\n";
}


void pronto_vis::getMeshInCircleIndices(pcl::PolygonMesh::Ptr meshin_ptr,
		   vector<double> &center, double radius,
		   vector<int> &polygon_in_circle_indices){
  int N_polygons = meshin_ptr->polygons.size ();
  pcl::PointCloud<pcl::PointXYZRGB> bigcloud;  
  pcl::fromPCLPointCloud2(meshin_ptr->cloud, bigcloud);
  Eigen::Vector4f tmp;
  for(size_t i=0; i< N_polygons; i++){ // each triangle/polygon
    pcl::Vertices apoly_in = meshin_ptr->polygons[i];
    int N_points = apoly_in.vertices.size ();
    for(size_t j=0; j< N_points; j++){ // each point
      uint32_t pt = apoly_in.vertices[j];
      tmp = bigcloud.points[pt].getVector4fMap(); // xyz 
      // are any points of the polygon in the circle, if so add them to the new model
      // currently only in 2d
      
      double dist2d = sqrt(pow(tmp(0) - center[0],2) + pow(tmp(1) - center[1],2));
      if (dist2d < radius){
	int in_mesh = (int)i; // inside 3D box
	polygon_in_circle_indices.push_back(in_mesh);
	break;
      }
    }
  }
  cout << polygon_in_circle_indices.size() << " is the polygon_in_circle_indices size\n";
}


void pronto_vis::getMeshInBox(pcl::PolygonMesh::Ptr meshin_ptr,
		   vector<double> &center, vector<double> &dgrid,
		   pcl::PolygonMesh::Ptr &minimesh_ptr){
  int N_polygons = meshin_ptr->polygons.size ();
  pcl::PointCloud<pcl::PointXYZRGB> bigcloud;  
  pcl::fromPCLPointCloud2(meshin_ptr->cloud, bigcloud);
  Eigen::Vector4f tmp;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr minicloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  int N_cloud_pts=0;
  for(size_t i=0; i< N_polygons; i++){ // each triangle/polygon
    pcl::Vertices apoly_in = meshin_ptr->polygons[i];
    int N_points = apoly_in.vertices.size ();
    
    bool add =0;
    for(size_t j=0; j< N_points; j++){ // each point
      uint32_t pt = apoly_in.vertices[j];
      tmp = bigcloud.points[pt].getVector4fMap(); // xyz 
      
      // are any points of the polygon in the box, add them to the new model
      if ((tmp(0) > center[0] - dgrid[0])&& (tmp(0) < center[0] + dgrid[0])) // widtin x
	if ((tmp(1) > center[1] - dgrid[1])&& (tmp(1) < center[1] + dgrid[1])) // within y
	  if ((tmp(2) > center[2] - dgrid[2])&& (tmp(2) < center[2] + dgrid[2])) // within z
	     add=1;	   // inside 3D box
	     
      if (add)
	break;
    }
    
    if (add){
      pcl::Vertices apoly_out;
      for(size_t j=0; j< N_points; j++){ // each point
	uint32_t pt = apoly_in.vertices[j];
	tmp = bigcloud.points[pt].getVector4fMap(); // xyz 
	pcl::PointXYZRGB a_pt= bigcloud.points[pt];
	minicloud->points.push_back(a_pt);
	apoly_out.vertices.push_back(N_cloud_pts);
	N_cloud_pts++;
      }
      minimesh_ptr->polygons.push_back(apoly_out);
    }
  } 
  minicloud->width    = N_cloud_pts;
  minicloud->height   = 1;
  minicloud->is_dense = true;
  minicloud->points.resize (minicloud->width * minicloud->height); // is this necessary?
  cout << minicloud->points.size() << " is the cloud size\n";
  cout << minimesh_ptr->polygons.size() << " is the polygon size\n";
  pcl::toPCLPointCloud2 (*minicloud, minimesh_ptr->cloud);
}
//#endif


///////////////////////////////////
//time x y z qx qy qz qw - all floating points
void read_poses_csv(std::string poses_files, std::vector<Isometry3dTime>& poses){
  int counter=0;
  string line;
  ifstream myfile (poses_files.c_str());
  if (myfile.is_open()){
    while ( myfile.good() ){
      getline (myfile,line);
      if (line.size() > 4){
	double quat[4];
	double pos[3];
	
	int64_t dtime;
 	sscanf(line.c_str(),"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
           &dtime,
           &pos[0],&pos[1],&pos[2],
   	   &quat[1],&quat[2],&quat[3],&quat[0]); // NBNBN: note the order
	
	Eigen::Quaterniond quat2(quat[0],quat[1],quat[2],quat[3]);
	Eigen::Isometry3d pose;
	pose.setIdentity();
	pose.translation()  << pos[0] ,pos[1],pos[2];
	pose.rotate(quat2);
	
	 Isometry3dTime poseT(dtime, pose);

	counter++;
 	poses.push_back(poseT);
      }
    }
    myfile.close();
  } else{
    printf( "Unable to open poses file\n%s",poses_files.c_str());
    return;
  }    
}

// display_tic_toc: a helper function which accepts a set of 
// timestamps and displays the elapsed time between them as 
// a fraction and time used [for profiling]
void display_tic_toc(std::vector<int64_t> &tic_toc,const string &fun_name){
  int tic_toc_size = tic_toc.size();
  
  double percent_tic_toc_last = 0;
  double dtime = ((double) (tic_toc[tic_toc_size-1] - tic_toc[0])/1000000);
  cout << "fraction_" << fun_name << ",";  
  for (int i=0; i < tic_toc_size;i++){
    double percent_tic_toc = (double) (tic_toc[i] - tic_toc[0])/(tic_toc[tic_toc_size-1] - tic_toc[0]);
    cout <<  percent_tic_toc - percent_tic_toc_last << ", ";
    percent_tic_toc_last = percent_tic_toc;
  }
  cout << "\ntime_" << fun_name << ",";
  double time_tic_toc_last = 0;
  for (int i=0; i < tic_toc_size;i++){
    double percent_tic_toc = (double) (tic_toc[i] - tic_toc[0])/(tic_toc[tic_toc_size-1] - tic_toc[0]);
    cout <<  percent_tic_toc*dtime - time_tic_toc_last << ", ";
    time_tic_toc_last = percent_tic_toc*dtime;
  }
  cout << "\ntotal_time_" << fun_name << " " << dtime << "\n";  
}


// Copied from kinect-lcm
static inline void
_matrix_vector_multiply_3x4_4d (const double m[12], const double v[4],
        double result[3])
{
    result[0] = m[0]*v[0] + m[1]*v[1] + m[2] *v[2] + m[3] *v[3];
    result[1] = m[4]*v[0] + m[5]*v[1] + m[6] *v[2] + m[7] *v[3];
    result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
}
