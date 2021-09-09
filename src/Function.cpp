#include <Glob_def.h>

float Norm(std::vector<float> x1)
{  
  float temp=0;
  float out;
  for (const auto &i : x1)
  {
    temp += i*i;
  }
  return sqrt(temp);
}

float dot(std::vector<float> v0, std::vector<float> v1)
{
  float out = 0;
  for (int i = 0; i < v0.size(); i++)
  {
    out += v0[i] * v1[i];
  }
  return out;
}

std::vector<float> emult(std::vector<float> v0, std::vector<float> v1)
{
  std::vector<float> out;
  for (int i = 0; i < v0.size(); i++)
  {
    out.push_back(v0[i] * v1[i]);
  }
  return out;
}

std::vector<float> mult_var(std::vector<float> v0, float v1)
{
  std::vector<float> out;
  for (const auto &elem : v0)
  {
    out.push_back(elem * v1);
  }
  return out;
}

std::vector<float> eplus(std::vector<float> v0, std::vector<float> v1)
{
  std::vector<float> out;
  for (int i = 0; i < v0.size(); i++)
  {
    out.push_back(v0[i] + v1[i]);
  }
  return out;
}

std::vector<float> eminus(std::vector<float> v0, std::vector<float> v1)
{
  std::vector<float> out;
  for (int i = 0; i < v0.size(); i++)
  {
    out.push_back(v0[i] - v1[i]);
  }
  return out;
}

std::vector<std::vector<float>> Multiply(std::vector<std::vector<float>> aMat,
                                         std::vector<std::vector<float>> bMat)
{
  std::vector<std::vector<float>> out;
  if (aMat[0].size() == bMat.size())
  {
    for (int col = 0; col < aMat.size(); col++)
    {
      for (int row = 0; row < bMat[0].size(); row++)
      {
        for (int inner = 0; inner < bMat.size(); inner++)
        {
          out[col][row] += aMat[col][inner] * bMat[inner][row];
        }
      }
    }
    return out;
  }
  return out;
}

std::vector<double> llh2ecef(std::vector<double> llh)
{
  // http://mathforum.org/library/drmath/view/51832.html
  // http://what-when-how.com/gps-with-high-rate-sensors/ecef-coordinate-systems-gps/
  std::vector<double> out;

  double lat = DEF_D2R(llh[0]); // latitude [Deg]
  double lon = DEF_D2R(llh[1]); // longitude [Deg]
  double h = llh[2];   // height [m]

  double a = 6378137.0;                 // the equatorial earth radius
  double b = 6356752.31424518;        // the polar cross-section earth radius
  double f = (a - b) / a;             // the "flattening" parameter
  double e = sqrt( (2*f) - (f*f)); // eccentricity e of the figure of the earth

  double C = 1 / sqrt( 1 - ((e*e) * (sin(lat)*sin(lat))) );
  double S = C * ( (1-f)*(1-f) );

  double x = (a*C + h) * cos(lat) * cos(lon);
  double y = (a*C + h) * cos(lat) * sin(lon);
  double z = (a*S + h) * sin(lat);

  out.push_back(x);
  out.push_back(y);
  out.push_back(z);

  std::cout<<"ECEF "<<x<<" "<<y<<" "<<z<<std::endl;

  return out;
}

std::vector<double> ecef2llh(std::vector<double> ecef)
{
  // LLH = Latitude[rad], Longitude[rad], Height[m]
  double x = ecef[0];
  double y = ecef[1];
  double z = ecef[2];

  double a = 6378137;                    // the equatorial earth radius
  double b = 6356752.31424518;           // the polar cross - section earth radius
  double f = (a - b) / a;                // the "flattening" parameter
  double e = sqrt(2.0f * f - pow(f, 2)); // eccentricity e of the figure of the earth

  double e_b = sqrt((pow(a, 2) - pow(b, 2)) / pow(b, 2));
  double p = sqrt(pow(x, 2) + pow(y, 2));
  double seta = atan2(z * a, p * b);

  double lon = atan2(y, x);
  double lat = atan2(z + pow(e_b, 2) * b * pow(sin(seta), 3), p - pow(e, 2) * a * pow(cos(seta), 3));
  double h = (p / cos(lat)) - (a / sqrt(1 - pow(e, 2) * pow(sin(lat), 2)));

  std::vector<double> llh{lat, lon, h};
  return llh;
}

std::vector<float> ecef2enu(std::vector<double> ecef,
                            std::vector<double> ori_ecef)
{
  std::vector<float> out;

  // ori_ecef is criterion point
  std::vector<double> ori_llh = ecef2llh(ori_ecef);
  double seta = ori_llh[0] - DEF_PI / 2; // Latitude[rad]
  double lam = -ori_llh[1] - DEF_PI / 2; // Longitude [rad]
  std::vector<double> diff;
  diff.push_back(ecef[0]-ori_ecef[0]);
  diff.push_back(ecef[1]-ori_ecef[1]);
  diff.push_back(ecef[2]-ori_ecef[2]);

  double sL = sin(seta);
  double cL = cos(seta);
  double sB = sin(lam);
  double cB = cos(lam);
  double x = diff[0];
  double y = diff[1];
  double z = diff[2];

  float e = (cB * x) + (-sB * y);
  float n = (cL * sB * x) + (cL * cB * y) + (-sL * z);
  float u = (sL * sB * x) + (sL * cB * y) + (cL * z);

  out.push_back(e);
  out.push_back(n);
  out.push_back(u);
  return out;
}

std::vector<float> ecef2en(std::vector<double> ecef,
                            std::vector<double> ori_ecef)
{
  std::vector<float> out;

  // ori_ecef is criterion point
  std::vector<double> ori_llh = ecef2llh(ori_ecef);
  double seta = ori_llh[0] - DEF_PI / 2; // Latitude[rad]
  double lam = -ori_llh[1] - DEF_PI / 2; // Longitude [rad]
  std::vector<double> diff;
  diff.push_back(ecef[0]-ori_ecef[0]);
  diff.push_back(ecef[1]-ori_ecef[1]);
  diff.push_back(ecef[2]-ori_ecef[2]);

  double sL = sin(seta);
  double cL = cos(seta);
  double sB = sin(lam);
  double cB = cos(lam);
  double x = diff[0];
  double y = diff[1];
  double z = diff[2];

  float e = (cB * x) + (-sB * y);
  float n = (cL * sB * x) + (cL * cB * y) + (-sL * z);
  float u = (sL * sB * x) + (sL * cB * y) + (cL * z);

  out.push_back(e);
  out.push_back(n);
  //out.push_back(u);
  std::cout<<"ENU "<<e<<" "<<n<<" "<<u<<std::endl;

  return out;
}

float PointDist(std::vector<float> P1, std::vector<float> P2, std::vector<float> N1)
{
  // Input
  // P1(x,y) ,  P2(x,y) ,  N1(x,y)
  std::vector<float> Vs = eminus(P2, P1);
  float nVs = Norm(Vs);
  std::vector<float> vec = mult_var(Vs, 1 / nVs);

  float L = dot(vec, eminus(N1, P1));

  std::vector<float> temp1 = mult_var(vec, L);
  std::vector<float> temp2 = eminus(P1, N1);
  std::vector<float> temp3 = eplus(temp1, temp2);

  return Norm(temp3);
}

std::vector<std::vector<float>> Outer(std::vector<float> C1, std::vector<float> C2)
{
  // Input
  // C1(x,y,r) ,  C2(x,y,r)

  std::vector<float> temp(2,0);
  std::vector<std::vector<float>> out(4,temp);

  /*std::cout<<C1[0]<<" "<<C1[1]<<" "<<C1[2]<<" "<<std::endl;
  std::cout<<C2[0]<<" "<<C2[1]<<" "<<C2[2]<<" "<<std::endl;//*/

  float r = C1[2];
  float r_ = C2[2];
  std::vector<float> v = eminus(C2, C1);
  float dist = Norm(v);

  float r_dif = r_ - r;

  // 원의 중심 간의 각도
  float seta = atan2(v[1], v[0]);
  float phi = asin(r_dif / dist);

  float th1 = seta + phi + (DEF_PI / 2);
  float th2 = seta - phi - (DEF_PI / 2);
  //std::cout<<"th: "<<th1<<" "<<th2<<std::endl;

  temp[0]= r * cos(th1) + C1[0];
  temp[1]= r * sin(th1) + C1[1];
  out[0]= temp;

  temp[0]= r_ * cos(th1) + C2[0];
  temp[1]= r_ * sin(th1) + C2[1];
  out[1]= temp;

  temp[0]= r * cos(th2) + C1[0];
  temp[1]= r * sin(th2) + C1[1];
  out[2]= temp;

  temp[0]= r_ * cos(th2) + C2[0];
  temp[1]= r_ * sin(th2) + C2[1];
  out[3]= temp;

  /*std::cout << out[1][0] << " " << out[1][1] << " "
      << out[0][0] << " " << out[0][1] << " "
      << out[2][0] << " " << out[2][1] << " "
      << out[3][0] << " " << out[3][1] <<std::endl; //*/

  return out;
}

std::vector<std::vector<float>> F_Circle(std::vector<float> C1, std::vector<float> P1, std::vector<float> P2)
{
  // Input
  // C1(x,y,r) ,  P1(x,y) ,  P2(x,y)
  std::vector<float> temp;
  std::vector<std::vector<float>> path;
  float r = C1[2];
  std::vector<float> Vs = eminus(P1, C1);
  std::vector<float> Ve = eminus(P2, C1);

  float As = atan2(Vs[1], Vs[0]);
  float Ae = atan2(Ve[1], Ve[0]);

  ROS_INFO("Circle Angle: %f, %f", As, Ae );
  if (As <= Ae){
    if (As >= -DEF_PI*0.5){
      for (float i=As; i<=Ae; i+=DEF_PI/180){
        temp.clear();
        temp.push_back(r*cos(i)+C1[0]);
        temp.push_back(r*sin(i)+C1[1]);
        path.push_back(temp);
      }
    }
    else{
      for (float i=As; i<=Ae-(DEF_PI*2); i-=DEF_PI/180){
        temp.clear();
        temp.push_back(r*cos(i)+C1[0]);
        temp.push_back(r*sin(i)+C1[1]);
        path.push_back(temp);
      }
    }
  }
  else{
    if (As <= (DEF_PI * 0.5)){
      for (float i=As; i>=Ae; i-=DEF_PI/180){
        temp.clear();
        temp.push_back(r * cos(i) + C1[0]);
        temp.push_back(r * sin(i) + C1[1]);
        path.push_back(temp);
      }
    }
    else{
      for (float i=As; i>=Ae+(DEF_PI*2); i+=DEF_PI/180){
        temp.clear();
        temp.push_back(r * cos(i) + C1[0]);
        temp.push_back(r * sin(i) + C1[1]);
        path.push_back(temp);
      }
    }
  }
  //ROS_INFO("Finish Circle Path, Sz: %i", (int)path.size() );
  return path;
}

bool ChkRange(std::vector<float> P1, std::vector<float> P2, std::vector<float> N1){
  // Input
  // P1(x,y) ,  P2(x,y) ,  N1(x,y)

  // X-axis
  if (P1[0] < P2[0]){
    if ((P1[0] < N1[0]) && (N1[0] < P2[0]))
      return true;
  }
  else if (P1[0] > P2[0]){
    if ((P1[0] > N1[0]) && (N1[0] > P2[0]))
      return true;
  }

  // Y-axis
  if (P1[1] < P2[1]){
    if ((P1[1] < N1[1]) && (N1[1] < P2[1]))
      return true;
  }
  else if (P1[1] > P2[1]){
    if ((P1[1] > N1[1]) && (N1[1] > P2[1]))
      return true;
  }

  return false;
}

std::vector<std::vector<float>> MakePath(std::vector<float> P1, std::vector<float> P2, std::vector<float> N1, float R){
  // Input
  // P1(x,y): Start Point
  // P2(x,y): End Point
  // N1(x,y): Center of No drone
  // R(r)   : Radius of No drone

  std::vector<std::vector<float>> out;
  std::vector<std::vector<float>> temp1, temp2, temp3;
  std::vector<float> var1, var2;

  if (ChkRange(P1, P2, N1)){
    float d = PointDist(P1, P2, N1);
    if(d <= R){
      ROS_INFO("Run Avoidance : Dist: %f, %f", d, R);
      var1.clear(); var2.clear();
      var1 = P1; var1.push_back(0);
      var2 = N1; var2.push_back(R);
      temp1 = Outer(var1, var2);

      var1.clear(); var2.clear();
      var1 = N1; var1.push_back(R);
      var2 = P2; var2.push_back(0);
      temp2 = Outer(var1, var2);

      var1.clear(); var2.clear();
      var1 = eminus(temp1[1],temp2[0]);
      float cost1 = Norm(var1);
      var2 = eminus(temp1[3],temp2[2]);
      float cost2 = Norm(var2);

      var1.clear(); var2.clear();
      if (cost1<=cost2){
        ROS_INFO("CW Avoidance");
        var1 = N1; var1.push_back(R);
        temp3 = F_Circle(var1, temp1[1],temp2[0]);
        /*std::cout << temp1[1][0] << " " << temp1[1][1] << " "
                  << temp1[0][0] << " " << temp1[0][1] << std::endl;//*/
        out.push_back(temp1[0]);
        out.push_back(temp1[1]);
        out.insert(out.end(), temp3.begin(), temp3.end());
        out.push_back(temp2[0]);
        out.push_back(temp2[1]);
        /*std::cout<<out.size()<<temp1[0].size()<<
        temp2[0].size()<<temp2[1].size()<<std::endl; //*/
      }
      else{
        ROS_INFO("CCW Avoidance");
        var1 = N1; var1.push_back(R);
        temp3 = F_Circle(var1, temp1[3],temp2[2]);
        out.push_back(temp1[2]);
        out.push_back(temp1[3]);
        out.insert(out.end(), temp3.begin(), temp3.end());
        out.push_back(temp2[2]);
        out.push_back(temp2[3]);
      }
    }
    else{
      ROS_INFO("No Avoidance 1 : Dist: %f, %f", d, R);
      out.push_back(P1);
      out.push_back(P2);
    }
  }
  else{
      ROS_INFO("No Avoidance 2");
      out.push_back(P1);
      out.push_back(P2);
  }

  ROS_INFO("finish MakePath, sz: %i", (int)out.size());
  return out;
}

float dist_pose(std::vector<float> P1, geometry_msgs::PoseStamped P2){
  std::vector<float> temp1 {0,0}, temp2;
  float dist;
  temp1[0]=P1[0]-P2.pose.position.x;
  temp1[1]=P1[1]-P2.pose.position.y;
  dist = Norm(temp1);
  return dist;
}