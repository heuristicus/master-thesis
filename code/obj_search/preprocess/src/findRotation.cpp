#include <simple_xml_parser.h>


int main(int argc, char *argv[]) {
    SimpleXMLParser<pcl::PointXYZRGB> parser;
    SimpleXMLParser<pcl::PointXYZRGB>::RoomData roomData = parser.loadRoomFromXML(argv[1]);
        
    return 0;
}
