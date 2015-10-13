#include <ros/ros.h>
#include <mongo/client/dbclient.h>

#include <sensor_msgs/CompressedImage.h>

using namespace mongo;

DBClientConnection *mongodb_conn;
std::string collection;
mongo::DBClientConnection c;

void run() {
  c.connect("localhost");
}

std::string intToString(int i)
{
    std::stringstream ss;
    std::string s;
    ss << i;
    s = ss.str();

    return s;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "image");

    try {
      run();
      std::cout << "connected ok" << std::endl;
    } catch( const mongo::DBException &e ) {
      std::cout << "caught " << e.what() << std::endl;
    }

  auto_ptr<DBClientCursor> cursor = c.query("roslog.cam1_image_raw_compressed", BSONObj());
  int count = 0;
  while (cursor->more()){
      count++;
      BSONObj p = cursor->next();
      BSONElement data = p.getField("data");
      int len;
      const char* pointer = p.getField("data").binDataClean(len);
      char arrs[len];
      memcpy(arrs ,pointer, len);
      char name[100];
      sprintf (name, "/home/alessandro/Scrivania/images/%i.jpeg", count);
      cout<<name;
      FILE *file = fopen(name, "w+");
      fwrite (arrs , 1 , len ,file );
      fclose(file);
  }

  cout << "fine"<< endl;
  return 0;


  ros::spin();

  delete mongodb_conn;

  return 0;
}
