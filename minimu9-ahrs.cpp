// For efficiency in compiling, this is the only file that uses the
// Eigen lirbary and the 'vector' type we made from it.

#include "vector.h"
#include "version.h"
#include "prog_options.h"
#include "minimu9.h"
#include "exceptions.h"
#include "pacer.h"
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <system_error>
#include <chrono>

// TODO: print warning if accelerometer magnitude is not close to 1 when starting up

FILE *open_filename(char *filename)
{
   FILE *new_handle = NULL;

   if (filename)
   {
      bool bNetwork = false;
      int sfd = -1, socktype;

      if(!strncmp("tcp://", filename, 6))
      {
         bNetwork = true;
         socktype = SOCK_STREAM;
      }
      else if(!strncmp("udp://", filename, 6))
      {
         bNetwork = true;
         socktype = SOCK_DGRAM;
      }

      if(bNetwork)
      {
         unsigned short port;
         filename += 6;
         char *colon;
         if(NULL == (colon = strchr(filename, ':')))
         {
            fprintf(stderr, "%s is not a valid IPv4:port, use something like tcp://1.2.3.4:1234 or udp://1.2.3.4:1234\n",
                    filename);
            exit(132);
         }
         if(1 != sscanf(colon + 1, "%hu", &port))
         {
            fprintf(stderr,
                    "Port parse failed. %s is not a valid network file name, use something like tcp://1.2.3.4:1234 or udp://1.2.3.4:1234\n",
                    filename);
            exit(133);
         }
         char chTmp = *colon;
         *colon = 0;

         struct sockaddr_in saddr={};
         saddr.sin_family = AF_INET;
         saddr.sin_port = htons(port);
         if(0 == inet_aton(filename, &saddr.sin_addr))
         {
            fprintf(stderr, "inet_aton failed. %s is not a valid IPv4 address\n",
                    filename);
            exit(134);
         }
         *colon = chTmp;

         if(0 <= (sfd = socket(AF_INET, socktype | SOCK_NONBLOCK, 0)))
         {
           fprintf(stderr, "Connecting to %s:%hu...", inet_ntoa(saddr.sin_addr), port);

           int iTmp = 1;
           while ((-1 == (iTmp = connect(sfd, (struct sockaddr *) &saddr, sizeof(struct sockaddr_in)))) && (EINTR == errno))
             ;
           if (iTmp < 0)
             fprintf(stderr, "error: %s\n", strerror(errno));
           else
             fprintf(stderr, "connected, sending video...\n");
         }
         else
           fprintf(stderr, "Error creating socket: %s\n", strerror(errno));

         if (sfd >= 0)
            new_handle = fdopen(sfd, "w");
      }
      else
      {
         new_handle = fopen(filename, "wb");
      }
   }

   return new_handle;
}  

// An Euler angle could take 8 chars: -234.678, but usually we only need 6.
float field_width = 6;
#define FLOAT_FORMAT std::fixed << std::setprecision(3) << std::setw(field_width)

std::ostream & operator << (std::ostream & os, const vector & vector)
{
  return os << FLOAT_FORMAT << vector(0) << ' '
            << FLOAT_FORMAT << vector(1) << ' '
            << FLOAT_FORMAT << vector(2);
}

std::ostream & operator << (std::ostream & os, const matrix & matrix)
{
  return os << (vector)matrix.row(0) << ' '
            << (vector)matrix.row(1) << ' '
            << (vector)matrix.row(2);
}

std::ostream & operator << (std::ostream & os, const quaternion & quat)
{
  return os << FLOAT_FORMAT << quat.w() << ' '
            << FLOAT_FORMAT << quat.x() << ' '
            << FLOAT_FORMAT << quat.y() << ' '
            << FLOAT_FORMAT << quat.z();
}

typedef void rotation_output_function(quaternion & rotation, std::stringstream& ss);

void output_quaternion(quaternion & rotation, std::stringstream& ss)
{
  ss << rotation;
}

void output_matrix(quaternion & rotation, std::stringstream& ss)
{
  ss << rotation.toRotationMatrix();
}

void output_euler(quaternion & rotation, std::stringstream& ss)
{
  ss << (vector)(rotation.toRotationMatrix().eulerAngles(2, 1, 0)
                        * (180 / M_PI));
}

void stream_raw_values(imu & imu)
{
  imu.enable();
  while(1)
  {
    imu.read_raw();
    printf("%7d %7d %7d  %7d %7d %7d  %7d %7d %7d\n",
           imu.m[0], imu.m[1], imu.m[2],
           imu.a[0], imu.a[1], imu.a[2],
           imu.g[0], imu.g[1], imu.g[2]
      );
    usleep(20*1000);
  }
}

//! Uses the acceleration and magnetic field readings from the compass
// to get a noisy estimate of the current rotation matrix.
// This function is where we define the coordinate system we are using
// for the ground coords:  North, East, Down.
matrix rotation_from_compass(const vector & acceleration, const vector & magnetic_field)
{
  vector down = -acceleration;     // usually true
  vector east = down.cross(magnetic_field); // actually it's magnetic east
  vector north = east.cross(down);

  east.normalize();
  north.normalize();
  down.normalize();

  matrix r;
  r.row(0) = north;
  r.row(1) = east;
  r.row(2) = down;
  return r;
}

typedef void fuse_function(quaternion & rotation, float dt, const vector & angular_velocity,
  const vector & acceleration, const vector & magnetic_field);

void fuse_compass_only(quaternion & rotation, float dt, const vector& angular_velocity,
  const vector & acceleration, const vector & magnetic_field)
{
  // Implicit conversion of rotation matrix to quaternion.
  rotation = rotation_from_compass(acceleration, magnetic_field);
}

// Uses the given angular velocity and time interval to calculate
// a rotation and applies that rotation to the given quaternion.
// w is angular velocity in radians per second.
// dt is the time.
void rotate(quaternion & rotation, const vector & w, float dt)
{
  // Multiply by first order approximation of the
  // quaternion representing this rotation.
  rotation *= quaternion(1, w(0)*dt/2, w(1)*dt/2, w(2)*dt/2);
  rotation.normalize();
}

void fuse_gyro_only(quaternion & rotation, float dt, const vector & angular_velocity,
  const vector & acceleration, const vector & magnetic_field)
{
  rotate(rotation, angular_velocity, dt);
}

void fuse_default(quaternion & rotation, float dt, const vector & angular_velocity,
  const vector & acceleration, const vector & magnetic_field)
{
  vector correction = vector(0, 0, 0);

  if (fabs(acceleration.norm() - 1) <= 0.3)
  {
    // The magnetidude of acceleration is close to 1 g, so
    // it might be pointing up and we can do drift correction.

    const float correction_strength = 1;

    matrix rotation_compass = rotation_from_compass(acceleration, magnetic_field);
    matrix rotation_matrix = rotation.toRotationMatrix();

    correction = (
      rotation_compass.row(0).cross(rotation_matrix.row(0)) +
      rotation_compass.row(1).cross(rotation_matrix.row(1)) +
      rotation_compass.row(2).cross(rotation_matrix.row(2))
      ) * correction_strength;

  }

  rotate(rotation, angular_velocity + correction, dt);
}

void ahrs(imu & imu, fuse_function * fuse, rotation_output_function * output, FILE *output_handle,
          FILE *output_nb_handle)
{
  imu.load_calibration();
  imu.enable();
  imu.measure_offsets();

  // The quaternion that can convert a vector in body coordinates
  // to ground coordinates when it its changed to a matrix.
  quaternion rotation = quaternion::Identity();

  // Set up a timer that expires every 20 ms.
  pacer loop_pacer;
  loop_pacer.set_period_ns(20000000);

  auto start = std::chrono::steady_clock::now();
  auto ref_start = std::chrono::steady_clock::now();
  while(1)
  {
    auto last_start = start;
    start = std::chrono::steady_clock::now();
    std::chrono::nanoseconds duration = start - last_start;
    float dt = duration.count() / 1e9;
    if (dt < 0){ throw std::runtime_error("Time went backwards."); }

    vector angular_velocity = imu.read_gyro();
    vector acceleration = imu.read_acc();
    vector magnetic_field = imu.read_mag();

    fuse(rotation, dt, angular_velocity, acceleration, magnetic_field);

    std::stringstream ss;
    std::chrono::nanoseconds timestamp = std::chrono::steady_clock::now() - ref_start;
    float timestamp_sec = timestamp.count() / 1e9;
    ss << timestamp_sec << "  ";
    output(rotation, ss);
    ss << "  " << acceleration << "  " << magnetic_field << std::endl;

    const std::string string_to_write = ss.str();
    const int string_length = string_to_write.length();
    int bytes_written = string_length;
    // Output streams (blocking and non-blocking)
    if (output_handle)
    {
      bytes_written = fwrite(string_to_write.c_str(), 1, string_length, output_handle);
      fflush(output_handle);
    }
    if (output_nb_handle)
    {
      fwrite(string_to_write.c_str(), 1, string_length, output_nb_handle);
      fflush(output_nb_handle);
    }
    
    // Check blocking write
    if (bytes_written != string_length)
    {
      std::cerr << "Blocking write was not successful" << std::endl;
    }
    loop_pacer.pace();
  }
}

int main_with_exceptions(int argc, char **argv)
{
  prog_options options = get_prog_options(argc, argv);

  if(options.show_help)
  {
    print_command_line_options_desc();
    std::cout << "For more information, run: man minimu9-ahrs" << std::endl;
    return 0;
  }

  if (options.show_version)
  {
    std::cout << VERSION << std::endl;
    return 0;
  }

  // Decide what sensors we want to use.
  sensor_set set;
  set.mag = set.acc = set.gyro = true;

  minimu9::comm_config config = minimu9::auto_detect(options.i2c_bus_name);

  sensor_set missing = set - minimu9::config_sensor_set(config);
  if (missing)
  {
    if (missing.mag)
    {
      std::cerr << "Error: No magnetometer found." << std::endl;
    }
    if (missing.acc)
    {
      std::cerr << "Error: No accelerometer found." << std::endl;
    }
    if (missing.gyro)
    {
      std::cerr << "Error: No gyro found." << std::endl;
    }
    std::cerr << "Error: Needed sensors are missing." << std::endl;
    return 1;
  }

  config = minimu9::disable_redundant_sensors(config, set);

  minimu9::handle imu;
  imu.open(config);

  rotation_output_function * output;

  // Figure out the output mode.
  if (options.output_mode == "matrix")
  {
    output = &output_matrix;
  }
  else if (options.output_mode == "quaternion")
  {
    output = &output_quaternion;
  }
  else if (options.output_mode == "euler")
  {
    field_width += 2;  // See comment above for field_width.
    output = &output_euler;
  }
  else
  {
    std::cerr << "Unknown output mode '" << options.output_mode << "'" << std::endl;
    return 1;
  }

  FILE *output_handle = NULL;
  if (!options.output_file.empty())
  {
    output_handle = open_filename((char *) options.output_file.c_str());
  }

  FILE *output_nb_handle = NULL;
  if (!options.output_file_nb.empty())
  {
    output_nb_handle = open_filename((char *) options.output_file_nb.c_str());
  }
  
  // Figure out the basic operating mode and start running.
  if (options.mode == "raw")
  {
    stream_raw_values(imu);
  }
  else if (options.mode == "gyro-only")
  {
    ahrs(imu, &fuse_gyro_only, output, output_handle, output_nb_handle);
  }
  else if (options.mode == "compass-only")
  {
    ahrs(imu, &fuse_compass_only, output, output_handle, output_nb_handle);
  }
  else if (options.mode == "normal")
  {
    ahrs(imu, &fuse_default, output, output_handle, output_nb_handle);
  }
  else
  {
    std::cerr << "Unknown mode '" << options.mode << "'" << std::endl;
    return 1;
  }
  return 0;
}

int main(int argc, char ** argv)
{
  try
  {
    main_with_exceptions(argc, argv);
  }
  catch(const std::system_error & error)
  {
    std::string what = error.what();
    const std::error_code & code = error.code();
    std::cerr << "Error: " << what << " (" << code << ")" << std::endl;
    return 2;
  }
  catch(const std::exception & error)
  {
    std::cerr << "Error: " << error.what() << std::endl;
    return 9;
  }
}
