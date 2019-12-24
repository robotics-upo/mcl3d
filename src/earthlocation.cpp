/*
    <Class that saves an earth location with latitude, longitude and altitude from sea level data.>
    Copyright (C) <year>  <name of author>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#include "mcl3d/earthlocation.h"
#include <sstream>
#include <math.h>


#include <kml/engine.h>
#include <kml/convenience/convenience.h>
#include <kml/base/date_time.h>
using kmldom::KmlFactory;
using kmldom::PointPtr;
using kmldom::PlacemarkPtr;
using kmldom::LineStringPtr;
using kmldom::CoordinatesPtr;
using kmldom::KmlPtr;
using kmlengine::KmlFile;
using kmlengine::KmlFilePtr;

using namespace std;

const double EarthLocation::equatorial_shift = 111319.9;
const double EarthLocation::latitude_shift = 111111.1;

EarthLocation::EarthLocation(double lat, double lon, double alt, double time, double yaw, double roll, double pitch)
{
	init(lat, lon, alt, time, yaw, roll, pitch);
}



void EarthLocation::init(double lat, double lon, double alt, double time, double yaw, double roll, double pitch)
{
	represent_time = false;
	this->lat = lat;
	this->lon = lon;
	this->altitude = alt;
	this->time = time;
	this->yaw = yaw;
	this->roll = roll;
	this->pitch = pitch;
}

EarthLocation::EarthLocation(const vector<double>& v)
{

  init(v);
}

void EarthLocation::init(const vector<double>& v_)
{
  vector<double> v = v_;
  for (unsigned int i = v.size();i < 7; i++) {
    v.push_back(0.0);
  }
  init(v.at(0), v.at(1), v.at(2), v.at(3), v.at(4), v.at(5), v.at(6));
}

#ifdef USE_KML

EarthLocation::EarthLocation(kmldom::ElementPtr e)
{
  init(e);
}

bool EarthLocation::init(kmldom::ElementPtr e)
{
  const kmldom::PlacemarkPtr place = kmldom::AsPlacemark(e);
  
  if (place == NULL) {
    return false;
  }
  const kmldom::TimePrimitivePtr time_prim = place->get_timeprimitive();
  kmldom::TimeStampPtr t = kmldom::AsTimeStamp(time_prim);
  
  if (t != NULL) {
//     cout << "Time catched.\n";
    kmlbase::DateTime *dt = kmlbase::DateTime::Create(t->get_when());
    
    if (dt != NULL) {
      time_t ti = dt->GetTimeT();
    
      cout << t->get_when() << "\t Ti = " << ti <<endl;
      
      
      time = ti;
      represent_time = true;
    }
  }
  
  const kmldom::GeometryPtr g = place->get_geometry();
  if (g == NULL) {
    return false;
  }
  if (g->Type() == kmldom::Type_Model) {
    const kmldom::ModelPtr m = kmldom::AsModel(g);    
    const kmldom::OrientationPtr o = m->get_orientation();
    yaw = o->get_heading();
    roll = o->get_roll();
    pitch = o->get_tilt();
    
    const kmldom::LocationPtr l = m->get_location();
    altitude = l->get_altitude();
    lat = l->get_latitude();
    lon = l->get_longitude();
  }

}

//! TODO: include the orientation and altitude (create placemarks with model geometry)
kmldom::PlacemarkPtr EarthLocation::getKMLPlacemark(const string &name) const
{
  PlacemarkPtr place = kmlconvenience::CreatePointPlacemark(name, lat, lon);
  
  return place;
}

PlacemarkPtr EarthLocation::getKMLLine(const string& name, const EarthLocation& end, kmldom::KmlFactory* factory) const
{
  CoordinatesPtr coords = factory->CreateCoordinates();
  LineStringPtr linestring = factory->CreateLineString();

  coords->add_latlngalt(getLatitude(), getLongitude(), getAltitude());
  coords->add_latlngalt(end.getLatitude(), end.getLongitude(), end.getAltitude());
  
  linestring->set_coordinates(coords);
  
  // Create the placemark containing the linestring
  PlacemarkPtr ret = factory->CreatePlacemark();
  ret->set_geometry(linestring);
  ret->set_name(name);
  ret->set_styleurl("#linestylemap");
    
  return ret;
}

#endif

void EarthLocation::shift(double north, double east, double alt, double time)
{
	double longitude_shift = equatorial_shift * cos(M_PI/180*lat);
	
	lon += east / longitude_shift;
	lat += north / latitude_shift;
	altitude +=  alt;
	this->time += time;
}

double EarthLocation::distance(const EarthLocation &e) 
{
  std::vector<double> v = toRelative(e);
  return sqrt(v.at(0)*v.at(0) + v.at(1)*v.at(1));
}

std::string EarthLocation::toString(char separator) const
{
	ostringstream os;
	
	if (represent_time) {
	  os << time << "\t";
	}
	
	os.precision(15);
    os << lat << separator << " " << lon << " " << altitude;
	
	return os.str();
}

string EarthLocation::toMatlab() const
{
  ostringstream os;
  os.precision(15);
  
  os << lat << " ";
  os << lon << " ";
  os << altitude << " ";
  os << yaw << " ";
  os << roll << " ";
  os << pitch;
  
  if (represent_time) {
    os << " " << time;
  }
  
  return os.str();
}

string EarthLocation::toMatlab(const EarthLocation &center) const
{
  ostringstream os;
  
  os.precision(15);
  
  os << (lon - center.lon) * equatorial_shift * cos(center.lat) << " ";
  os << (lat - center.lat) * latitude_shift << " ";
  os << altitude << " ";
  os << yaw << " ";
  os << roll << " ";
  os << pitch;
  
  if (represent_time) {
    os << " " << time;
  }
  
  return os.str();
}


EarthLocation EarthLocation::interpolate(const EarthLocation &post, double time) {
  EarthLocation ret = *this;
  
  double mult = (time - this->time)/(post.time - this->time);
  
  ret.altitude += (post.altitude - this->altitude ) * mult;
  ret.lat += (post.lat - this->lat ) * mult;
  ret.lon += (post.lon - this->lon ) * mult;
  ret.time += (post.time - this->time) * mult;
  ret.yaw += (post.yaw - this->yaw) * mult;
  ret.roll += (post.roll - this->roll) * mult;
  ret.yaw += (post.yaw - this->yaw) * mult;
  
  return ret;
}

// We suppose ENU transform
std::vector<double> EarthLocation::toRelative(const EarthLocation& center) const
{
  std::vector<double> ret;
  double inc_east = (lon - center.lon) * equatorial_shift * cos( (center.lat + lat) / 2/ 180*M_PI);
  double inc_north = (lat - center.lat) * latitude_shift;
  double inc_alt = altitude - center.altitude ;
  
  ret.push_back(inc_east);
  ret.push_back(inc_north);
  ret.push_back(inc_alt);
  
  return ret; 
}

void EarthLocation::fromRelative(const vector< double >& v_, const EarthLocation& e)
{
  *this = e;
  vector <double> v = v_;
  if (v.size() < 2) 
    return;
  if (v.size() == 2) {
    v.push_back(0);
  }
  shift(v.at(1), v.at(0), v.at(2)); // North is component 1 and east is component 0
}

