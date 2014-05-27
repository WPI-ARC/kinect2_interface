using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;

//////////////////////////////////////////////////
/////    AUTOGENERATED MESSAGE DEFINITION    /////
//////////////////////////////////////////////////
/////         DO NOT MODIFY BY HAND!         /////
//////////////////////////////////////////////////

namespace ROS_CS
{
    namespace geometry_msgs
    {
        public class PolygonStamped : ROS_CS.Core.BaseMessage
        {
            public readonly string typeID = "PolygonStamped";
            public readonly string md5sum = "c6be8f7dc3bee7fe9e8d296070f53340";
            public std_msgs.Header header;
            public geometry_msgs.Polygon polygon;

            public PolygonStamped ()
            {
                header = new std_msgs.Header();
                polygon = new geometry_msgs.Polygon();
            }

            public override string ToString ()
            {
                return ROS_CS.Core.PrettyPrinter.PrettyPrint(ToStringRaw());
            }

            public override string ToStringRaw ()
            {
                string string_rep = typeID + ":\n";
                string_rep += header.ToStringRaw() + "\n";
                string_rep += polygon.ToStringRaw() + "\n";
                return string_rep;
            }

            public override void Serialize(MemoryStream stream)
            {
                header.Serialize(stream);
                polygon.Serialize(stream);
            }

            public override int Deserialize(System.Byte[] serialized)
            {
                return Deserialize(serialized, 0);
            }

            public override int Deserialize(System.Byte[] serialized, int startIndex)
            {
                int curIndex = startIndex;
                curIndex += header.Deserialize(serialized, curIndex);
                curIndex += polygon.Deserialize(serialized, curIndex);
                return (curIndex - startIndex);
            }

        }
    }
}
