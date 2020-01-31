/*
    <one line to give the program's name and a brief idea of what it does.>
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

#ifndef EARTHLOCATION_H
#define EARTHLOCATION_H

#include <string>
#include <vector>
#ifdef USE_KML
#include <kml/dom.h>
#endif

//! @brief This class is used to represent an Earth Location in terms of longitude and latitude.
//! Uses decimal degrees as internal representation of latitude and longitude.
//! The altitude is stored in meters above the sea level.
//! ToString method gives many representations.
class EarthLocation
{
	public:
		//! @brief Default constructor. Creates a Location with latitude longitude and altitude
		//! @param lat Latitude (degrees)
		//! @param lon Longitude (degrees)
		//! @param alt Altitude (meters)
		//! @param time Time (seconds)
        //! @param yaw Yaw angle (rads)
		//! @param roll Roll angle (rads)
		//! @param pitch Pitch (rads)
		EarthLocation(double lat = 0.0, double lon = 0.0, double alt = 0.0, double time = 0.0
                      , double yaw = 0.0, double roll = 0.0, double pitch = 0.0);
		
		EarthLocation(const std::vector<double> &v);

#ifdef USE_KML
		EarthLocation(kmldom::ElementPtr e);
#endif
		
		//! @brief Shifts the position of this location. The distance to shift is given in meters.
		//! @param north Shift distance in north direction
		//! @param east Shift distance in east direction
		//! @param time Shift time (seconds)
		void shift(double north = 0.0, double east = 0.0, double alt = 0.0, double time = 0.0);
		
		//! @brief Calculates the cartesian distance between two locations
		//! @param e The other location
		//! @return The planar distance
		double distance(const EarthLocation &e);		
		
		//! @brief Represents the content of the class in deg min sec way
		//! @return The string that represents the contents
		virtual std::string toString(char separator =',') const;
		
		//! @brief Represents the content of the class in a MATLAB row
		//! @NOTE format is: latitude longitude altitude yaw roll pitch
		//! @NOTE All angles are represented in degrees
		//! @return The string that represents the contents
		virtual std::string toMatlab() const;
		
		//! @brief Represents the content of the class in a MATLAB row
		//! @NOTE format is: x (related to longitude) y(related to latitude) altitude yaw roll pitch
		//! @NOTE All angles are represented in degrees
		//! @return The string that represents the contents
		virtual std::string toMatlab(const EarthLocation &center) const;
#ifdef USE_KML
		//! @brief Gets the a KML point placemark
		//! @param name The name of the placemark 
		//! @return A pointer to the coordinates.
		kmldom::PlacemarkPtr getKMLPlacemark(const std::string &name) const;
                
        //! @brief Gets the a KML line placemark from this to the end
        //! @param name The name of the placemark 
        //! @param end THe end point of the line
        //! @param factory a KmlFactory properly initialized
        //! @return A pointer to the coordinates.
        kmldom::PlacemarkPtr getKMLLine(const string& name, const EarthLocation& end, kmldom::KmlFactory* factory) const;
#endif
		
		//! @brief Gets the longitude coordinate in degrees
		//! @return The coordinate
		inline double getLongitude() const {return lon;}
		
		//! @brief Gets the latitude coordinate in degrees
		//! @return The coordinate
		inline double getLatitude() const {return lat;}
		
		//! @brief Gets the altitude coordinate
		//! @return The coordinate
		inline double getAltitude() const {return altitude;};
		
		//! @brief Gets the altitude coordinate
		//! @return The coordinate
		inline double getTime() const {return time;};
		
		inline void setLatitude(double lati) {lat = lati;}
		inline void setLongitude(double longi) {lon = longi;}
		inline void setAltitude(double alt) {altitude = alt;}
		
		//! @brief Interpolates the state with another in a det time
		//! @param post The other state
		//! @param time The instant of time where the interpolation will be performed
		//! @return The interpolated state
		EarthLocation interpolate(const EarthLocation &post, double time);
    
		//! @brief Translates absolute coordinates into relative coordinates
		//! @return A vector whose first component is the north deviation, second the east deviation and finally the altitude (m)
		std::vector<double> toRelative(const EarthLocation &center) const;
                
		void fromRelative(const std::vector< double >& v_, const EarthLocation& e);
		
		bool represent_time; // If true, the time is added

	protected:
		double lon; // Stores the longitude in degrees
		double lat; // Stores the latitude in degrees
		double altitude; // Stores the altitude in meters above the sea level
		double time; // Time when the location was reached (optional)
		
		double pitch;
		double yaw;
		double roll;
		
		//! @brief Initializes the class with latitude and longitude in decimal degrees
		void init(double lat, double lon = 0.0, double alt = 0.0, double time = 0.0, 
			  double yaw = 0.0, double roll = 0.0, double pitch = 0.0);
		
		void init(const std::vector<double> &v);
#ifdef USE_KML
		bool init(kmldom::ElementPtr e);
#endif
		
		static const double equatorial_shift;
		static const double latitude_shift;
};

#endif // EARTHLOCATION_H
