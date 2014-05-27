using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace Kinect2ROS
{
    class Program
    {
        private static ROS_CS.SocketBridge.SocketTX<ROS_CS.sensor_msgs.Image> color_tx;
        private static ROS_CS.SocketBridge.SocketTX<ROS_CS.sensor_msgs.Image> depth_tx;
        //private static ROS_CS.SocketBridge.SocketRX<ROS_CS.sensor_msgs.Image> color_rx;
        private static KinectSensor kinect;
        private static ColorFrameReader color_reader;
        private static DepthFrameReader depth_reader;
        private static TimeSpan unix_base_ts;
        private static readonly long windows_epoch_to_unix_epoch_seconds = 11644473600;
        private static readonly long ticks_per_second = 10000000;
        private static readonly long unix_base_ticks = windows_epoch_to_unix_epoch_seconds * ticks_per_second;

        static void Main(string[] args)
        {
            unix_base_ts = new TimeSpan(unix_base_ticks);
            // Initialize the kinect sensor
            //kinect IS NOT AN INSTANTIABLE OBJECT - you get a 'device' from the system driver
            try
            {
                kinect = KinectSensor.Default;
            }
            catch (ArgumentOutOfRangeException)
            {
                Console.WriteLine("It appears that no Kinect sensor is connected to your computer!");
                return;
            }
            //device initialization code
            try
            {
                Console.WriteLine("Starting initialization");
                kinect.Open();
                color_reader = kinect.ColorFrameSource.OpenReader();
                depth_reader = kinect.DepthFrameSource.OpenReader();
                Console.WriteLine("Initialization successful");
            }
            catch (InvalidOperationException)
            {
                Console.WriteLine("Failed trying to set up the kinect. Is the device connected, on, and not being used by another application?");
                return;
            }
            // Initialize the socket bridge
            color_tx = new ROS_CS.SocketBridge.SocketTX<ROS_CS.sensor_msgs.Image>(9001);
            depth_tx = new ROS_CS.SocketBridge.SocketTX<ROS_CS.sensor_msgs.Image>(9002);
            //color_rx = new ROS_CS.SocketBridge.SocketRX<ROS_CS.sensor_msgs.Image>("127.0.0.1", 9001, ColorImageCB);
            bool control = true;
            long depth_timestamp = 0;
            long color_timestamp = 0;
            while (control)
            {
                //Get depth data
                DepthFrame latest_depth = depth_reader.AcquireLatestFrame();
                if (latest_depth != null)
                {
                    Console.WriteLine("Current depth timestamp: {0:D}, new timestamp: {1:D}", depth_timestamp, latest_depth.RelativeTime);
                    if (latest_depth.RelativeTime > depth_timestamp)
                    {
                        ROS_CS.sensor_msgs.Image depth_image = GetDepthImageFromRaw(latest_depth);
                        if (depth_image != null)
                        {
                            Console.WriteLine("Sending depth image");
                            depth_tx.Send(depth_image);
                        }
                        else
                        {
                            Console.WriteLine("Null depth image");
                        }
                        depth_timestamp = latest_depth.RelativeTime;
                    }
                }
                //Get color data
                ColorFrame latest_color = color_reader.AcquireLatestFrame();
                if (latest_color != null)
                {
                    Console.WriteLine("Current color timestamp: {0:D}, new timestamp: {1:D}", color_timestamp, latest_color.RelativeTime);
                    if (latest_color.RelativeTime > color_timestamp)
                    {
                        ROS_CS.sensor_msgs.Image color_image = GetColorImageFromRaw(latest_color);
                        if (color_image != null)
                        {
                            Console.WriteLine("Sending color image");
                            color_tx.Send(color_image);
                        }
                        else
                        {
                            Console.WriteLine("Null color image");
                        }
                        color_timestamp = latest_color.RelativeTime;
                    }
                }
            }
        }

        static void ColorImageCB(ROS_CS.sensor_msgs.Image msg)
        {
            Console.WriteLine("Received image message");
            Console.WriteLine("Width: {0:D}\nHeight: {1:D}\nStep: {2:D}\nData length (real): {3:D}\nShould be: {4:D}", msg.width, msg.height, msg.step, msg.data.Count, (1920 * 1080 * 4));
            //Console.WriteLine(msg);
        }

        static ROS_CS.sensor_msgs.Image GetColorImageFromRaw(ColorFrame new_color_frame)
        {
            ROS_CS.sensor_msgs.Image color_image = new ROS_CS.sensor_msgs.Image();
            color_image.header.frame_id = "kinect2_color_optical_frame";
            color_image.header.stamp = KinectTimestampsToROS(new_color_frame.RelativeTime);
            color_image.is_bigendian = 0;
            color_image.height = (uint)new_color_frame.FrameDescription.Height;
            color_image.width = (uint)new_color_frame.FrameDescription.Width;
            color_image.step = (uint)new_color_frame.FrameDescription.Width * 4;
            color_image.encoding = "rgba8";
            byte[] color_data = new byte[color_image.step * color_image.height];
            new_color_frame.CopyConvertedFrameDataToArray(color_data, ColorImageFormat.Rgba);
            color_image.data.AddRange(color_data);
            return color_image;
        }

        static ROS_CS.sensor_msgs.Image GetDepthImageFromRaw(DepthFrame new_depth_frame)
        {
            ROS_CS.sensor_msgs.Image depth_image = new ROS_CS.sensor_msgs.Image();
            depth_image.header.frame_id = "kinect2_depth_optical_frame";
            depth_image.header.stamp = KinectTimestampsToROS(new_depth_frame.RelativeTime);
            depth_image.is_bigendian = 0;
            depth_image.height = (uint)new_depth_frame.FrameDescription.Height;
            depth_image.width = (uint)new_depth_frame.FrameDescription.Width;
            depth_image.step = (uint)new_depth_frame.FrameDescription.Width * 2;
            depth_image.encoding = "mono16";
            ushort[] depth_data = new ushort[new_depth_frame.FrameDescription.Height * new_depth_frame.FrameDescription.Width];
            new_depth_frame.CopyFrameDataToArray(depth_data);
            foreach (ushort depth in depth_data)
            {
                ushort cleaned_depth = (ushort)(depth >> 3);
                byte high_byte = (byte)((cleaned_depth & 0xFF00) >> 8);
                byte low_byte = (byte)(cleaned_depth & 0x00FF);
                depth_image.data.Add(high_byte);
                depth_image.data.Add(low_byte);
            }
            return depth_image;
        }

        static ROS_CS.Core.Time KinectTimestampsToROS(long kinect_timestamp)
        {
            ROS_CS.Core.Time timestamp = new ROS_CS.Core.Time();
            TimeSpan ts = new TimeSpan(kinect_timestamp);
            TimeSpan unix_ts = ts.Subtract(unix_base_ts);
            timestamp.secs = unix_ts.Seconds;
            timestamp.nsecs = unix_ts.Milliseconds * 1000000;
            return timestamp;
        }
    }
}
