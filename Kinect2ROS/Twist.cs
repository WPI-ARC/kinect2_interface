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
        public class Twist : ROS_CS.Core.BaseMessage
        {
            public readonly string typeID = "Twist";
            public readonly string md5sum = "9f195f881246fdfa2798d1d3eebca84a";
            public geometry_msgs.Vector3 linear;
            public geometry_msgs.Vector3 angular;

            public Twist ()
            {
                linear = new geometry_msgs.Vector3();
                angular = new geometry_msgs.Vector3();
            }

            public override string ToString ()
            {
                return ROS_CS.Core.PrettyPrinter.PrettyPrint(ToStringRaw());
            }

            public override string ToStringRaw ()
            {
                string string_rep = typeID + ":\n";
                string_rep += linear.ToStringRaw() + "\n";
                string_rep += angular.ToStringRaw() + "\n";
                return string_rep;
            }

            public override void Serialize(MemoryStream stream)
            {
                linear.Serialize(stream);
                angular.Serialize(stream);
            }

            public override int Deserialize(System.Byte[] serialized)
            {
                return Deserialize(serialized, 0);
            }

            public override int Deserialize(System.Byte[] serialized, int startIndex)
            {
                int curIndex = startIndex;
                curIndex += linear.Deserialize(serialized, curIndex);
                curIndex += angular.Deserialize(serialized, curIndex);
                return (curIndex - startIndex);
            }

        }
    }
}
