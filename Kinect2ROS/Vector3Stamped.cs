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
        public class Vector3Stamped : ROS_CS.Core.BaseMessage
        {
            public readonly string typeID = "Vector3Stamped";
            public readonly string md5sum = "7b324c7325e683bf02a9b14b01090ec7";
            public std_msgs.Header header;
            public geometry_msgs.Vector3 vector;

            public Vector3Stamped ()
            {
                header = new std_msgs.Header();
                vector = new geometry_msgs.Vector3();
            }

            public override string ToString ()
            {
                return ROS_CS.Core.PrettyPrinter.PrettyPrint(ToStringRaw());
            }

            public override string ToStringRaw ()
            {
                string string_rep = typeID + ":\n";
                string_rep += header.ToStringRaw() + "\n";
                string_rep += vector.ToStringRaw() + "\n";
                return string_rep;
            }

            public override void Serialize(MemoryStream stream)
            {
                header.Serialize(stream);
                vector.Serialize(stream);
            }

            public override int Deserialize(System.Byte[] serialized)
            {
                return Deserialize(serialized, 0);
            }

            public override int Deserialize(System.Byte[] serialized, int startIndex)
            {
                int curIndex = startIndex;
                curIndex += header.Deserialize(serialized, curIndex);
                curIndex += vector.Deserialize(serialized, curIndex);
                return (curIndex - startIndex);
            }

        }
    }
}
