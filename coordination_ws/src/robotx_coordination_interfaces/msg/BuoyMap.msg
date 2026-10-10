std_msgs/Header header
# Hash of explicit WGS84 water datum; receivers must verify this.
string reference_id
float64 origin_latitude
float64 origin_longitude
float64 water_altitude
SurveyBuoy[] buoys
SurveyFootprint[] footprints
