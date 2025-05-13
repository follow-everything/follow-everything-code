#include <teb_local_planner/map_process.h>

std::vector<std::vector<Point2D>> mapProcess::normalNvigation_findGoalLineGroups(costmap_2d::Costmap2D* costmap, std::pair<double, double> global_goal, const std::vector<geometry_msgs::PoseStamped>& local_plan){
  // backup costmap information
  shrink_dis_ = 20;   // Difference between large and small map
  costmap_ = costmap;
  width_large_ = costmap->getSizeInCellsX();
  height_large_ = costmap->getSizeInCellsY();
  resolution_large_ = costmap->getResolution();
  origin_x_large_ = costmap->getOriginX();
  origin_y_large_ = costmap->getOriginY();

  width_ = costmap_->getSizeInCellsX() - shrink_dis_ * 2;
  height_ = costmap_->getSizeInCellsY() - shrink_dis_ * 2;
  resolution_ = costmap_->getResolution();
  origin_x_ = costmap_->getOriginX() + shrink_dis_ * resolution_;
  origin_y_ = costmap_->getOriginY() + shrink_dis_ * resolution_;

  global_goal_ = global_goal;
  
  costmap_data_ = costmap_->getCharMap();

  // 求一个res，顺时针排列所有goal line group
  std::vector<std::vector<Point2D>> res;
  // 大costmap的goal
  int mx = (local_plan.back().pose.position.x-origin_x_large_)/resolution_large_;
  int my = (local_plan.back().pose.position.y-origin_y_large_)/resolution_large_;
  farther_goal_ = Point2D(mx,my);

  // judge motion type
  motion_type_ = "normal";

  // judge the type of goal extension
  std::string goal_extension_ = "";
  if (mx>=shrink_dis_-2 && mx<width_large_-shrink_dis_+2 && my>=shrink_dis_-2 && my<height_large_-shrink_dis_+2)
    goal_extension_ = "goal_approach";
  else
    goal_extension_ = "goal_line";
  assert(goal_extension_ == "goal_approach" || goal_extension_ == "goal_line");
  ROS_ERROR("%s", goal_extension_.c_str());

  // goal line
  if (goal_extension_ == "goal_line"){
    // goal line是在小local map的边界处
    std::vector<std::vector<bool>> map_potential_goal = std::vector<std::vector<bool>> (width_large_, std::vector<bool>(height_large_, false));
    for (int i=shrink_dis_; i<width_large_-shrink_dis_; i++){
      map_potential_goal[i][shrink_dis_] = true;
      map_potential_goal[i][height_large_-1-shrink_dis_] = true;
    }
    for (int j=shrink_dis_; j<height_large_-shrink_dis_; j++){
      map_potential_goal[shrink_dis_][j] = true;
      map_potential_goal[width_large_-1-shrink_dis_][j] = true;
    }
    // 遍历local_plan，在map_potential_goal中求得小local map边界与goal的交点，得到near_goal
    Point2D_float near_goal_float;
    for (auto p:local_plan){
      int mx = (p.pose.position.x-origin_x_large_)/resolution_large_;
      int my = (p.pose.position.y-origin_y_large_)/resolution_large_;
      if (checkInMap(mx,my,width_large_,height_large_) && map_potential_goal[mx][my]){
          near_goal_ = Point2D(mx,my);
          near_goal_float = Point2D_float(p.pose.position.x, p.pose.position.y);
          break; 
      }
    }
    // 在map_potential_goal中用曼哈顿距离求near_goal附近点，仅仅near_goal的goal_lin_half_length距离以内的点会被视作候选goal line点 （若离得太远则改成false，扔掉）
    int goal_line_half_length = std::min(int(1.1*hypot(global_goal_.first-near_goal_float.x, global_goal_.second-near_goal_float.y)/resolution_large_), int(0.3*(width_large_-2*shrink_dis_))) - 2 ;
    goal_line_half_length = std::max(1, goal_line_half_length );

    // int goal_line_half_length = std::max(1, int(0.3 * (width_large_-2*shrink_dis_)-2) );
    for (int x=0; x<width_large_; x++){
      for (int y=0; y<height_large_; y++){
        if (!map_potential_goal[x][y]) continue;
        // 计算曼哈顿距离
        int dis = std::abs(x-near_goal_.x) + std::abs(y-near_goal_.y);
        // 判断点(x,y)是否在near_goal附近
        if (dis >= goal_line_half_length)
          map_potential_goal[x][y] = false;
      }
    }
    // 遍历剩余的map_potential_goal，对值为true的所有点，求他们的x_min, x_max, y_min, y_max
    int x_min = width_large_-1, y_min = height_large_-1;
    int x_max = 0, y_max = 0;
    for (int i=0; i<width_large_; i++){
      for (int j=0; j<height_large_; j++){
        if (!map_potential_goal[i][j]) continue;
        x_min = std::min(x_min, i);
        y_min = std::min(y_min, j);
        x_max = std::max(x_max, i);
        y_max = std::max(y_max, j);
      }
    }
    // 用于记录goal_line的极限
    int goal_line_x_min=x_min, goal_line_x_max=x_max, goal_line_y_min=y_min, goal_line_y_max=y_max;
    // 以(width_*0.5, height_*0.5)为中心，判断father_goal所处的象限，把象限的角作用到min和max中，扩大后续从father_goal到goal_line的搜索范围
    if (farther_goal_.x<width_large_*0.5 && farther_goal_.y<height_large_*0.5){
      x_min = 0;
      y_min = 0;
    }
    else if (farther_goal_.x>width_large_*0.5 && farther_goal_.y<height_large_*0.5){
      x_max = width_large_-1;
      y_min = 0;
    }
    else if (farther_goal_.x<width_large_*0.5 && farther_goal_.y>height_large_*0.5){
      x_min = 0;
      y_max = height_large_-1;
    }
    else if (farther_goal_.x>width_large_*0.5 && farther_goal_.y>height_large_*0.5){
      x_max = width_large_-1;
      y_max = height_large_-1;
    }
    // 赋值三个map {大map，小map，goal map}，用于可视化
    map_boundaries_.push_back({{0,0}, {0,height_large_-1}, {width_large_-1,height_large_-1}, {width_large_-1,0}});
    map_boundaries_.push_back({{shrink_dis_,shrink_dis_}, {shrink_dis_,height_large_-1-shrink_dis_}, {width_large_-1-shrink_dis_,height_large_-1-shrink_dis_}, {width_large_-1-shrink_dis_,shrink_dis_}});
    map_boundaries_.push_back({{x_min,y_min}, {x_min,y_max}, {x_max,y_max}, {x_max,y_min}});

    // 提取局部小地图，由于(x_min,y_min), (y_min, y_max)
    std::vector<unsigned char> map_vector;
    // assign static obstacles
    for (unsigned int i = 0; i < width_large_*height_large_; i++)
      map_vector.push_back(costmap_data_[ i ]);

    // true则为obstacle
    int small_width = x_max-x_min+1;
    int small_height = y_max-y_min+1;
    std::vector<std::vector<bool>> map_small = std::vector<std::vector<bool>> (small_width, std::vector<bool>(small_height, false));
    for (int i=x_min; i<=x_max; i++){
      for (int j=y_min; j<=y_max; j++){
        if (static_cast<int>(map_vector[i+j*width_large_])<253){
          map_small[i-x_min][j-y_min] = true;
        }
      }
    }

    // 创建小地图small_map，在后续中赋值map_obstacle_labeled和obs_list
    int label_index = 1;
    std::vector<std::vector<int>> map_obstacle_labeled;
    std::map<int, std::vector<Point2D>> obs_list;
    map_obstacle_labeled = std::vector<std::vector<int>> (small_width, std::vector<int>(small_height));

    // 斜对角遍历2D矩阵
    for (int x_iter=0; x_iter<small_height+small_width; x_iter++){
      // x+y的和 为 x_iter
      int x = std::min(x_iter, (int)small_width-1);
      int y = std::max((int)x_iter-x, 0);
      // 和不变时，x逐渐减小，y逐渐增大
      while (x>=0 && y<small_height){
        // 左上的四邻接 检测是否可以connect
        if (map_small[x][y])
          addObsConnect(x,y,map_obstacle_labeled,obs_list, label_index,small_width,small_height);
        x--;
        y++;
      }
    }

    // 顺时针赋值all_goal_line_points，先找到四个可行边界 ..._maintain
    int father_x_in_map = farther_goal_.x-x_min;
    father_x_in_map = std::max(0,father_x_in_map);
    father_x_in_map = std::min(father_x_in_map, small_width-1);
    int father_y_in_map = farther_goal_.y-y_min;
    father_y_in_map = std::max(0,father_y_in_map);
    father_y_in_map = std::min(father_y_in_map, small_height-1);
    // ROS_INFO("(%d,%d): (%d,%d) - (%d,%d)", map_obstacle_labeled.size(), map_obstacle_labeled.front().size(), x_min,y_min, farther_goal_.x,farther_goal_.y);
    int farther_goal_list_index = map_obstacle_labeled[father_x_in_map][father_y_in_map];
    std::vector<Point2D> all_goal_line_points;
    std::vector<Point2D> y_max_maintain;
    std::vector<Point2D> x_max_maintain;
    std::vector<Point2D> y_min_maintain;
    std::vector<Point2D> x_min_maintain;
    // 注意costmap的坐标系，是以机器人当前朝向为正x的右手系
    for (int x = goal_line_x_min+1; x<=goal_line_x_max; x++){
      int y = goal_line_y_max;
      if (map_potential_goal[x][y]){
        y_max_maintain.push_back(Point2D(x,y));
      }
    }
    for (int y = goal_line_y_max-1; y>=goal_line_y_min; y--){
      int x = goal_line_x_max;
      if (map_potential_goal[x][y]){
        x_max_maintain.push_back(Point2D(x,y));
      }
    }
    for (int x = goal_line_x_max-1; x>=goal_line_x_min; x--){
      int y = goal_line_y_min;
      if (map_potential_goal[x][y]){
        y_min_maintain.push_back(Point2D(x,y));
      }
    }  
    for (int y = goal_line_y_min+1; y<=goal_line_y_max; y++){
      int x = goal_line_x_min;
      if (map_potential_goal[x][y]){
        x_min_maintain.push_back(Point2D(x,y));
      }
    }
    // 确定起点、顺序、叠加问题，共 四 * 2 种情况：
    // CASE1: father_goal的引入并未带来x_min和y_min的改变： 如果goal_line_x_min==goal_lin_x_max或goal_line_y_min==goal_lin_y_max，则只添加对应的就好，否则从y_min_maintain开始
    if (x_min==goal_line_x_min && y_min==goal_line_y_min){
      if (goal_line_x_min==goal_line_x_max){
        all_goal_line_points=x_max_maintain;
      }
      else if (goal_line_y_min==goal_line_y_max){
        all_goal_line_points=y_max_maintain;
      }
      else{
        all_goal_line_points.insert(all_goal_line_points.end(), x_min_maintain.begin(), x_min_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), y_max_maintain.begin(), y_max_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), x_max_maintain.begin(), x_max_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), y_min_maintain.begin(), y_min_maintain.end());
      }
    }
    // CASE2: father_goal的引入并未带来x_min和y_max变： 如果goal_line_x_min==goal_line_x_max或goal_line_y_min==goal_line_y_max，则只添加对应的就好，否则从x_min_maintain开始
    else if (x_min==goal_line_x_min && y_max==goal_line_y_max){
      if (goal_line_x_min==goal_line_x_max){
        all_goal_line_points=x_max_maintain;
      }
      else if (goal_line_y_min==goal_line_y_max){
        all_goal_line_points=y_min_maintain;
      }
      else{
        all_goal_line_points.insert(all_goal_line_points.end(), y_max_maintain.begin(), y_max_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), x_max_maintain.begin(), x_max_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), y_min_maintain.begin(), y_min_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), x_min_maintain.begin(), x_min_maintain.end());
      }
    }
    // CASE3: father_goal的引入并未带来x_max和y_min变： 如果goal_line_x_min==goal_line_x_max或goal_line_y_min==goal_line_y_max，则只添加对应的就好，否则从x_max_maintain开始
    else if(x_max==goal_line_x_max && y_min==goal_line_y_min){
      if (goal_line_x_min==goal_line_x_max)
        all_goal_line_points=x_min_maintain;
      else if (goal_line_y_min==goal_line_y_max)
        all_goal_line_points=y_max_maintain;
      else{
        all_goal_line_points.insert(all_goal_line_points.end(), y_min_maintain.begin(), y_min_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), x_min_maintain.begin(), x_min_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), y_max_maintain.begin(), y_max_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), x_max_maintain.begin(), x_max_maintain.end());
      }
    }
    // CASE4: father_goal的引入并未带来x_max和y_max变： 如果goal_line_x_min==goal_line_x_max或goal_line_y_min==goal_line_y_max，则只添加对应的就好，否则从y_max_maintain开始
    else if (x_max==goal_line_x_max && y_max==goal_line_y_max){
      if (goal_line_x_min==goal_line_x_max){
        all_goal_line_points=x_min_maintain;
      }
      else if (goal_line_y_min==goal_line_y_max){
        all_goal_line_points=y_min_maintain;
      }
      else{
        all_goal_line_points.insert(all_goal_line_points.end(), x_max_maintain.begin(), x_max_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), y_min_maintain.begin(), y_min_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), x_min_maintain.begin(), x_min_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), y_max_maintain.begin(), y_max_maintain.end());
      }
    }

    // 如果goal_line被障碍物截断，则把all_goal_line_points给拆成多份
    std::vector<Point2D> one_goal_line;
    // 找到四个角里在map_obstacle_labeled中的那个，设置为0，便于后续的截断  
    int x1=shrink_dis_-x_min, y1=shrink_dis_-y_min;
    int x2=width_large_-1-shrink_dis_-x_min, y2=height_large_-1-shrink_dis_-y_min;
    int temp_x,temp_y;
    if (0<=x1 && x1<small_width)
      temp_x = x1;
    else
      temp_x = x2;
    if (0<=y1 && y1<small_height)
      temp_y = y1;
    else
      temp_y = y2;
    // goal lines被障碍物或角截断
    for (auto iter=all_goal_line_points.begin(); iter!=all_goal_line_points.end(); iter++){
      if (map_obstacle_labeled[iter->x-x_min][iter->y-y_min] == farther_goal_list_index && !(iter->x-x_min==temp_x && iter->y-y_min==temp_y)){
        one_goal_line.push_back(*iter);
      }
      else{
        if(!one_goal_line.empty()){
          // 长度大于等于3的goal line才会被考虑，因为它会被收缩边界
          if (one_goal_line.size()>=3){
            one_goal_line.erase(one_goal_line.begin());
            one_goal_line.pop_back();
            res.push_back(one_goal_line);
          }
          one_goal_line.clear();
        }
      }
    }
    // 加入最后一个one_goal_line
    if(!one_goal_line.empty()){
      // 长度大于等于3的goal line才会被考虑，因为它会被收缩边界
      if (one_goal_line.size()>=3){
        one_goal_line.erase(one_goal_line.begin());
        one_goal_line.pop_back();
        res.push_back(one_goal_line);
      }
      // 到全局目标附近啦
      else{
        res.push_back(one_goal_line);
      }
      one_goal_line.clear();
    }

    // 将res转换到小地图下
    for (auto iter_o=res.begin(); iter_o!=res.end(); iter_o++){
      for (auto iter_i=iter_o->begin(); iter_i!=iter_o->end(); iter_i++){
        iter_i->x -= shrink_dis_;
        iter_i->y -= shrink_dis_;
      }
    }
    near_goal_.x = near_goal_.x - shrink_dis_;
    near_goal_.y = near_goal_.y - shrink_dis_;
    farther_goal_.x = farther_goal_.x - shrink_dis_;
    farther_goal_.y = farther_goal_.y - shrink_dis_;
    goal_line_lists_ = res;
    return res;
  }

  if (goal_extension_ == "goal_approach"){
    farther_goal_.x = farther_goal_.x - shrink_dis_;
    farther_goal_.y = farther_goal_.y - shrink_dis_;
    for (auto p:local_plan){
      int mx = (p.pose.position.x-origin_x_large_)/resolution_large_;
      int my = (p.pose.position.y-origin_y_large_)/resolution_large_;
      if (mx>=shrink_dis_ && mx<width_large_-shrink_dis_ && my>=shrink_dis_ && my<height_large_-shrink_dis_){
          near_goal_ = Point2D(mx-shrink_dis_,my-shrink_dis_);
      }
    }
    res.push_back({near_goal_});
    goal_line_lists_ = res;
    return res;
  }

}

std::vector<std::vector<Point2D>> mapProcess::followNvigation_findGoalLineGroups(costmap_2d::Costmap2D* costmap, std::pair<double, double> pedestrian_position, double safe_distance, double clear_distance, const std::vector<geometry_msgs::PoseStamped>& local_plan){
  // preprocess the costmap
  shrink_dis_ = 20;   // Difference between large and small map
  costmap_ = costmap;
  width_large_ = costmap->getSizeInCellsX();
  height_large_ = costmap->getSizeInCellsY();
  resolution_large_ = costmap->getResolution();
  origin_x_large_ = costmap->getOriginX();
  origin_y_large_ = costmap->getOriginY();

  width_ = costmap_->getSizeInCellsX() - shrink_dis_ * 2;
  height_ = costmap_->getSizeInCellsY() - shrink_dis_ * 2;
  resolution_ = costmap_->getResolution();
  origin_x_ = costmap_->getOriginX() + shrink_dis_ * resolution_;
  origin_y_ = costmap_->getOriginY() + shrink_dis_ * resolution_;

  global_goal_ = pedestrian_position;
  
  costmap_data_ = costmap_->getCharMap();

  // Convert pedestrian position (world coordinates) to costmap index
  int ped_x_index = static_cast<int>((pedestrian_position.first - origin_x_large_) / resolution_large_);
  int ped_y_index = static_cast<int>((pedestrian_position.second - origin_y_large_) / resolution_large_);

  // if (ped_x_index < 0 || ped_x_index >= width_large_ || ped_y_index < 0 || ped_y_index >= height_large_)
  //     ROS_WARN("Pedestrian position is out of costmap bounds.");

  // Compute safe_distance in terms of grid cells
  safe_distance_cells_ = static_cast<int>(safe_distance / resolution_large_);
  clear_distance_cells_ = static_cast<int>(clear_distance / resolution_large_);

  // Iterate over a square region around the pedestrian and set values to 0
  // for (int dy = -clear_distance_cells_; dy <= clear_distance_cells_; ++dy) {
  //     for (int dx = -clear_distance_cells_; dx <= clear_distance_cells_; ++dx) {
  //         int x = ped_x_index + dx;
  //         int y = ped_y_index + dy;

  //         // Ensure the point is within map bounds
  //         if (x >= 0 && x < width_large_ && y >= 0 && y < height_large_) {
  //             int index = y * width_large_ + x;

  //             // Compute Euclidean distance (optional, for circular area check)
  //             double dist = sqrt(dx * dx + dy * dy) * resolution_large_;
  //             if (dist <= safe_distance) {
  //                 costmap_data_[index] = 0;  // Set costmap value to 0
  //             }
  //         }
  //     }
  // }  


  // find goal lines

  // 求一个res，顺时针排列所有goal line group
  std::vector<std::vector<Point2D>> res;
  // 大costmap的goal
  int mx = (local_plan.back().pose.position.x-origin_x_large_)/resolution_large_;
  int my = (local_plan.back().pose.position.y-origin_y_large_)/resolution_large_;
  farther_goal_ = Point2D(mx,my);

  // judge the type of goal extension
  if (safe_distance==0)
    goal_extension_ = "goal_approach";
  else if ( (mx-width_large_*0.5)*(mx-width_large_*0.5) + (my-height_large_*0.5)*(my-height_large_*0.5) <= safe_distance_cells_*safe_distance_cells_ )
    goal_extension_ = "goal_back";
  else if (mx>=shrink_dis_-2 && mx<width_large_-shrink_dis_+2 && my>=shrink_dis_-2 && my<height_large_-shrink_dis_+2)
    goal_extension_ = "goal_circle";
  else
    goal_extension_ = "goal_line";
  assert(goal_extension_ == "goal_approach" || goal_extension_ == "goal_back" || goal_extension_ == "goal_circle" || goal_extension_ == "goal_line");
  ROS_ERROR("%s", goal_extension_.c_str());


  // CASE1 如果farther_goal在小local_map外，goal line
  if (goal_extension_ == "goal_line"){
    // goal line是在小local map的边界处
    std::vector<std::vector<bool>> map_potential_goal = std::vector<std::vector<bool>> (width_large_, std::vector<bool>(height_large_, false));
    for (int i=shrink_dis_; i<width_large_-shrink_dis_; i++){
      map_potential_goal[i][shrink_dis_] = true;
      map_potential_goal[i][height_large_-1-shrink_dis_] = true;
    }
    for (int j=shrink_dis_; j<height_large_-shrink_dis_; j++){
      map_potential_goal[shrink_dis_][j] = true;
      map_potential_goal[width_large_-1-shrink_dis_][j] = true;
    }
    // 遍历local_plan，在map_potential_goal中求得小local map边界与goal的交点，得到near_goal
    Point2D_float near_goal_float;
    for (auto p:local_plan){
      int mx = (p.pose.position.x-origin_x_large_)/resolution_large_;
      int my = (p.pose.position.y-origin_y_large_)/resolution_large_;
      if (checkInMap(mx,my,width_large_,height_large_) && map_potential_goal[mx][my]){
          near_goal_ = Point2D(mx,my);
          near_goal_float = Point2D_float(p.pose.position.x, p.pose.position.y);
          break; 
      }
    }
    // 在map_potential_goal中用曼哈顿距离求near_goal附近点，仅仅near_goal的goal_lin_half_length距离以内的点会被视作候选goal line点 （若离得太远则改成false，扔掉）
    int goal_line_half_length = std::min(int(1.1*hypot(global_goal_.first-near_goal_float.x, global_goal_.second-near_goal_float.y)/resolution_large_), int(0.3*(width_large_-2*shrink_dis_))) - 2 ;
    goal_line_half_length = std::max(1, goal_line_half_length );

    // int goal_line_half_length = std::max(1, int(0.3 * (width_large_-2*shrink_dis_)-2) );
    for (int x=0; x<width_large_; x++){
      for (int y=0; y<height_large_; y++){
        if (!map_potential_goal[x][y]) continue;
        // 计算曼哈顿距离
        int dis = std::abs(x-near_goal_.x) + std::abs(y-near_goal_.y);
        // 判断点(x,y)是否在near_goal附近
        if (dis >= goal_line_half_length)
          map_potential_goal[x][y] = false;
      }
    }
    // 遍历剩余的map_potential_goal，对值为true的所有点，求他们的x_min, x_max, y_min, y_max
    int x_min = width_large_-1, y_min = height_large_-1;
    int x_max = 0, y_max = 0;
    for (int i=0; i<width_large_; i++){
      for (int j=0; j<height_large_; j++){
        if (!map_potential_goal[i][j]) continue;
        x_min = std::min(x_min, i);
        y_min = std::min(y_min, j);
        x_max = std::max(x_max, i);
        y_max = std::max(y_max, j);
      }
    }
    // 用于记录goal_line的极限
    int goal_line_x_min=x_min, goal_line_x_max=x_max, goal_line_y_min=y_min, goal_line_y_max=y_max;
    // 以(width_*0.5, height_*0.5)为中心，判断father_goal所处的象限，把象限的角作用到min和max中，扩大后续从father_goal到goal_line的搜索范围
    if (farther_goal_.x<width_large_*0.5 && farther_goal_.y<height_large_*0.5){
      x_min = 0;
      y_min = 0;
    }
    else if (farther_goal_.x>width_large_*0.5 && farther_goal_.y<height_large_*0.5){
      x_max = width_large_-1;
      y_min = 0;
    }
    else if (farther_goal_.x<width_large_*0.5 && farther_goal_.y>height_large_*0.5){
      x_min = 0;
      y_max = height_large_-1;
    }
    else if (farther_goal_.x>width_large_*0.5 && farther_goal_.y>height_large_*0.5){
      x_max = width_large_-1;
      y_max = height_large_-1;
    }
    // 赋值三个map {大map，小map，goal map}，用于可视化
    map_boundaries_.push_back({{0,0}, {0,height_large_-1}, {width_large_-1,height_large_-1}, {width_large_-1,0}});
    map_boundaries_.push_back({{shrink_dis_,shrink_dis_}, {shrink_dis_,height_large_-1-shrink_dis_}, {width_large_-1-shrink_dis_,height_large_-1-shrink_dis_}, {width_large_-1-shrink_dis_,shrink_dis_}});
    map_boundaries_.push_back({{x_min,y_min}, {x_min,y_max}, {x_max,y_max}, {x_max,y_min}});

    // 提取局部小地图，由于(x_min,y_min), (y_min, y_max)
    std::vector<unsigned char> map_vector;
    // assign static obstacles
    for (unsigned int i = 0; i < width_large_*height_large_; i++)
      map_vector.push_back(costmap_data_[ i ]);

    // true则为obstacle
    int small_width = x_max-x_min+1;
    int small_height = y_max-y_min+1;
    std::vector<std::vector<bool>> map_small = std::vector<std::vector<bool>> (small_width, std::vector<bool>(small_height, false));
    for (int i=x_min; i<=x_max; i++){
      for (int j=y_min; j<=y_max; j++){
        if (static_cast<int>(map_vector[i+j*width_large_])<253){
          map_small[i-x_min][j-y_min] = true;
        }
      }
    }

    // 创建小地图small_map，在后续中赋值map_obstacle_labeled和obs_list
    int label_index = 1;
    std::vector<std::vector<int>> map_obstacle_labeled;
    std::map<int, std::vector<Point2D>> obs_list;
    map_obstacle_labeled = std::vector<std::vector<int>> (small_width, std::vector<int>(small_height));

    // 斜对角遍历2D矩阵
    for (int x_iter=0; x_iter<small_height+small_width; x_iter++){
      // x+y的和 为 x_iter
      int x = std::min(x_iter, (int)small_width-1);
      int y = std::max((int)x_iter-x, 0);
      // 和不变时，x逐渐减小，y逐渐增大
      while (x>=0 && y<small_height){
        // 左上的四邻接 检测是否可以connect
        if (map_small[x][y])
          addObsConnect(x,y,map_obstacle_labeled,obs_list, label_index,small_width,small_height);
        x--;
        y++;
      }
    }

    // 顺时针赋值all_goal_line_points，先找到四个可行边界 ..._maintain
    int father_x_in_map = farther_goal_.x-x_min;
    father_x_in_map = std::max(0,father_x_in_map);
    father_x_in_map = std::min(father_x_in_map, small_width-1);
    int father_y_in_map = farther_goal_.y-y_min;
    father_y_in_map = std::max(0,father_y_in_map);
    father_y_in_map = std::min(father_y_in_map, small_height-1);
    // ROS_INFO("(%d,%d): (%d,%d) - (%d,%d)", map_obstacle_labeled.size(), map_obstacle_labeled.front().size(), x_min,y_min, farther_goal_.x,farther_goal_.y);
    int farther_goal_list_index = map_obstacle_labeled[father_x_in_map][father_y_in_map];
    std::vector<Point2D> all_goal_line_points;
    std::vector<Point2D> y_max_maintain;
    std::vector<Point2D> x_max_maintain;
    std::vector<Point2D> y_min_maintain;
    std::vector<Point2D> x_min_maintain;
    // 注意costmap的坐标系，是以机器人当前朝向为正x的右手系
    for (int x = goal_line_x_min+1; x<=goal_line_x_max; x++){
      int y = goal_line_y_max;
      if (map_potential_goal[x][y]){
        y_max_maintain.push_back(Point2D(x,y));
      }
    }
    for (int y = goal_line_y_max-1; y>=goal_line_y_min; y--){
      int x = goal_line_x_max;
      if (map_potential_goal[x][y]){
        x_max_maintain.push_back(Point2D(x,y));
      }
    }
    for (int x = goal_line_x_max-1; x>=goal_line_x_min; x--){
      int y = goal_line_y_min;
      if (map_potential_goal[x][y]){
        y_min_maintain.push_back(Point2D(x,y));
      }
    }  
    for (int y = goal_line_y_min+1; y<=goal_line_y_max; y++){
      int x = goal_line_x_min;
      if (map_potential_goal[x][y]){
        x_min_maintain.push_back(Point2D(x,y));
      }
    }
    // 确定起点、顺序、叠加问题，共 四 * 2 种情况：
    // CASE1: father_goal的引入并未带来x_min和y_min的改变： 如果goal_line_x_min==goal_lin_x_max或goal_line_y_min==goal_lin_y_max，则只添加对应的就好，否则从y_min_maintain开始
    if (x_min==goal_line_x_min && y_min==goal_line_y_min){
      if (goal_line_x_min==goal_line_x_max){
        all_goal_line_points=x_max_maintain;
      }
      else if (goal_line_y_min==goal_line_y_max){
        all_goal_line_points=y_max_maintain;
      }
      else{
        all_goal_line_points.insert(all_goal_line_points.end(), x_min_maintain.begin(), x_min_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), y_max_maintain.begin(), y_max_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), x_max_maintain.begin(), x_max_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), y_min_maintain.begin(), y_min_maintain.end());
      }
    }
    // CASE2: father_goal的引入并未带来x_min和y_max变： 如果goal_line_x_min==goal_line_x_max或goal_line_y_min==goal_line_y_max，则只添加对应的就好，否则从x_min_maintain开始
    else if (x_min==goal_line_x_min && y_max==goal_line_y_max){
      if (goal_line_x_min==goal_line_x_max){
        all_goal_line_points=x_max_maintain;
      }
      else if (goal_line_y_min==goal_line_y_max){
        all_goal_line_points=y_min_maintain;
      }
      else{
        all_goal_line_points.insert(all_goal_line_points.end(), y_max_maintain.begin(), y_max_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), x_max_maintain.begin(), x_max_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), y_min_maintain.begin(), y_min_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), x_min_maintain.begin(), x_min_maintain.end());
      }
    }
    // CASE3: father_goal的引入并未带来x_max和y_min变： 如果goal_line_x_min==goal_line_x_max或goal_line_y_min==goal_line_y_max，则只添加对应的就好，否则从x_max_maintain开始
    else if(x_max==goal_line_x_max && y_min==goal_line_y_min){
      if (goal_line_x_min==goal_line_x_max)
        all_goal_line_points=x_min_maintain;
      else if (goal_line_y_min==goal_line_y_max)
        all_goal_line_points=y_max_maintain;
      else{
        all_goal_line_points.insert(all_goal_line_points.end(), y_min_maintain.begin(), y_min_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), x_min_maintain.begin(), x_min_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), y_max_maintain.begin(), y_max_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), x_max_maintain.begin(), x_max_maintain.end());
      }
    }
    // CASE4: father_goal的引入并未带来x_max和y_max变： 如果goal_line_x_min==goal_line_x_max或goal_line_y_min==goal_line_y_max，则只添加对应的就好，否则从y_max_maintain开始
    else if (x_max==goal_line_x_max && y_max==goal_line_y_max){
      if (goal_line_x_min==goal_line_x_max){
        all_goal_line_points=x_min_maintain;
      }
      else if (goal_line_y_min==goal_line_y_max){
        all_goal_line_points=y_min_maintain;
      }
      else{
        all_goal_line_points.insert(all_goal_line_points.end(), x_max_maintain.begin(), x_max_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), y_min_maintain.begin(), y_min_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), x_min_maintain.begin(), x_min_maintain.end());
        all_goal_line_points.insert(all_goal_line_points.end(), y_max_maintain.begin(), y_max_maintain.end());
      }
    }

    // 如果goal_line被障碍物截断，则把all_goal_line_points给拆成多份
    std::vector<Point2D> one_goal_line;
    // 找到四个角里在map_obstacle_labeled中的那个，设置为0，便于后续的截断  
    int x1=shrink_dis_-x_min, y1=shrink_dis_-y_min;
    int x2=width_large_-1-shrink_dis_-x_min, y2=height_large_-1-shrink_dis_-y_min;
    int temp_x,temp_y;
    if (0<=x1 && x1<small_width)
      temp_x = x1;
    else
      temp_x = x2;
    if (0<=y1 && y1<small_height)
      temp_y = y1;
    else
      temp_y = y2;
    // goal lines被障碍物或角截断
    for (auto iter=all_goal_line_points.begin(); iter!=all_goal_line_points.end(); iter++){
      if (map_obstacle_labeled[iter->x-x_min][iter->y-y_min] == farther_goal_list_index && !(iter->x-x_min==temp_x && iter->y-y_min==temp_y)){
        one_goal_line.push_back(*iter);
      }
      else{
        if(!one_goal_line.empty()){
          // 长度大于等于3的goal line才会被考虑，因为它会被收缩边界
          if (one_goal_line.size()>=3){
            one_goal_line.erase(one_goal_line.begin());
            one_goal_line.pop_back();
            res.push_back(one_goal_line);
          }
          one_goal_line.clear();
        }
      }
    }
    // 加入最后一个one_goal_line
    if(!one_goal_line.empty()){
      // 长度大于等于3的goal line才会被考虑，因为它会被收缩边界
      if (one_goal_line.size()>=3){
        one_goal_line.erase(one_goal_line.begin());
        one_goal_line.pop_back();
        res.push_back(one_goal_line);
      }
      // 到全局目标附近啦
      else{
        res.push_back(one_goal_line);
      }
      one_goal_line.clear();
    }

    // 将res转换到小地图下
    for (auto iter_o=res.begin(); iter_o!=res.end(); iter_o++){
      for (auto iter_i=iter_o->begin(); iter_i!=iter_o->end(); iter_i++){
        iter_i->x -= shrink_dis_;
        iter_i->y -= shrink_dis_;
      }
    }
    near_goal_.x = near_goal_.x - shrink_dis_;
    near_goal_.y = near_goal_.y - shrink_dis_;
    farther_goal_.x = farther_goal_.x - shrink_dis_;
    farther_goal_.y = farther_goal_.y - shrink_dis_;
    goal_line_lists_ = res;
    return res;
  }

  // CASE2 如果farther_goal在小local_map内，直接返回。考虑更大，但返回更早
  if (goal_extension_ == "goal_circle"){
    farther_goal_.x = farther_goal_.x - shrink_dis_;
    farther_goal_.y = farther_goal_.y - shrink_dis_;

    // initialize potential map
    std::vector<std::vector<bool>> map_potential_goal = std::vector<std::vector<bool>> (width_large_, std::vector<bool>(height_large_, false));
    // iterate the map_potential_goal, if the distance between a point (x,y) and the center (width_large_*0.5, height_large_*0.5) is smaller than safe_distance_cells_, then set map_potential_goal[x][y] = true (候选点)
    // iterate costmap_data_, if its value is >= 253, then set map_potential_goal[x][y] = false
    for (int x=0; x<width_large_; x++){
      for (int y=0; y<height_large_; y++){
        if (abs(hypot(x-mx,y-my)-safe_distance_cells_) <= 0.5 && static_cast<int>(costmap_data_[x+y*width_large_])<253)
          map_potential_goal[x][y] = true;
      }
    }
    for (int x=0; x<width_large_; x++){
      for (int y=0; y<height_large_; y++){
        if (map_potential_goal[x][y])
          goal_circle_.push_back(Point2D(x-shrink_dis_,y-shrink_dis_));
      }
    }

    for (auto p:local_plan){
      int mx = (p.pose.position.x-origin_x_large_)/resolution_large_;
      int my = (p.pose.position.y-origin_y_large_)/resolution_large_;
      if (mx>=shrink_dis_ && mx<width_large_-shrink_dis_ && my>=shrink_dis_ && my<height_large_-shrink_dis_){
          near_goal_ = Point2D(mx-shrink_dis_,my-shrink_dis_);
      }
    }
    res.push_back({near_goal_});
    goal_line_lists_ = res;
    return res;
  }

  // CASE3 如果robot is too close to the object
  if (goal_extension_ == "goal_back"){
    farther_goal_.x = farther_goal_.x - shrink_dis_;
    farther_goal_.y = farther_goal_.y - shrink_dis_;
    
    // initialize potential map
    std::vector<std::vector<bool>> map_potential_goal = std::vector<std::vector<bool>> (width_large_, std::vector<bool>(height_large_, false));
    // iterate the map_potential_goal, if the distance between a point (x,y) and the center (width_large_*0.5, height_large_*0.5) is smaller than safe_distance_cells_, then set map_potential_goal[x][y] = true (候选点)
    for (int x=0; x<width_large_; x++){
      for (int y=0; y<height_large_; y++){
        if (abs(hypot(x-mx,y-my)-safe_distance_cells_) <= 0.5)
          map_potential_goal[x][y] = true;
      }
    }

    int delete_range = 2;
    // iterate costmap_data_, if its value is >= 253, then set map_potential_goal[x][y] = false
    for (int x=0; x<width_large_; x++){
      for (int y=0; y<height_large_; y++){
        // if any nearby 2 cells is greater than 253, then set map_potential_goal[x][y] = false
        bool is_deleted = false;
        for (int i=-delete_range; i<=delete_range && !is_deleted; i++){
          for (int j=-delete_range; j<=delete_range && !is_deleted; j++){
            if ((x+i>=0 && x+i<width_large_) && (y+j>=0 && y+j<height_large_)){
              if (static_cast<int>(costmap_data_[x+i+(y+j)*width_large_])>=253){
                map_potential_goal[x][y] = false;
                is_deleted = true;
              }
            }
          }
        }
      }
    }

    for (int x=0; x<width_large_; x++){
      for (int y=0; y<height_large_; y++){
        if (map_potential_goal[x][y])
          goal_circle_.push_back(Point2D(x-shrink_dis_,y-shrink_dis_));
      }
    }
    
    // iterate the map_potential_goal, get the point (x,y) with value true and has the shortest distance from the robot (width_large_*0.5, height_large_*0.5)
    near_goal_ = Point2D(width_large_*0.5-shrink_dis_,height_large_*0.5-shrink_dis_);
    double min_dis = std::numeric_limits<double>::max();
    for (int x=0; x<width_large_; x++){
      for (int y=0; y<height_large_; y++){
        if (map_potential_goal[x][y]){
          double dis = hypot(x-width_large_*0.5,y-height_large_*0.5);
          if (dis < min_dis){
            min_dis = dis;
            near_goal_ = Point2D(x-shrink_dis_,y-shrink_dis_);
          }
        }
      }
    }
    res.push_back({near_goal_});
    goal_line_lists_ = res;
    return res;
  }

  // CASE4 if the object is out of robot's scope
  if (goal_extension_ == "goal_approach"){
    farther_goal_.x = farther_goal_.x - shrink_dis_;
    farther_goal_.y = farther_goal_.y - shrink_dis_;    
    for (auto p:local_plan){
      int mx = (p.pose.position.x-origin_x_large_)/resolution_large_;
      int my = (p.pose.position.y-origin_y_large_)/resolution_large_;
      if (mx>=shrink_dis_ && mx<width_large_-shrink_dis_ && my>=shrink_dis_ && my<height_large_-shrink_dis_){
          near_goal_ = Point2D(mx-shrink_dis_,my-shrink_dis_);
      }
    }

    // set motion type
    if (static_cast<int>(costmap_data_[near_goal_.x+shrink_dis_ + (near_goal_.y+shrink_dis_)*width_large_])<253)
      motion_type_ = "follow";

    res.push_back({near_goal_});
    goal_line_lists_ = res;
    return res;
  }

}

