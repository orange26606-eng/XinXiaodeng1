// ByteTrack两阶段关联策略移植，采用MIT许可，来源版本见配套记录。
// 来源为third_party/bytetrack/SOURCE.json指定的BYTETracker.cpp固定版本。
// 使用现有的可变 dt Kalman 状态和有界 LAPJV 分配。
// 不使用 Eigen/OpenCV 的 STrack 存储，未接入神经 ReID 或 BoT-SORT。
// 未匹配的检测框保留为可选的暂定轨迹，并可参与低分阶段关联。


static std::vector<int> byte_lap(const std::vector<std::vector<double>>&cost){
 size_t rows=cost.size(),cols=rows?cost[0].size():0;if(!rows||!cols)return std::vector<int>(rows,-1);size_t n=std::max(rows,cols);if(n>129)throw std::runtime_error("BYTE_LAP_CAPACITY");
 std::vector<double>matrix(n*n,1.);std::vector<double*>pointers(n);std::vector<int>x(n,-1),y(n,-1);for(size_t i=0;i<n;i++){pointers[i]=matrix.data()+i*n;for(size_t j=0;j<n;j++)if(i<rows&&j<cols)matrix[i*n+j]=cost[i][j];}
 if(objlapjv::lapjv_internal((unsigned)n,pointers.data(),x.data(),y.data())<0)throw std::runtime_error("BYTE_LAP_ALLOCATION_FAILED");x.resize(rows);for(int&j:x)if(j<0||(size_t)j>=cols)j=-1;return x;
}
template<class Edge>static std::vector<Edge>byte_association(const std::vector<Track>&tracks,const std::vector<Box>&boxes,const std::vector<Edge>&edges,int selected,const Frame&f,const Config&cfg){
 std::vector<Edge>matches;std::set<int>used_tracks,used_boxes;
 // 先恢复已确认目标身份，再执行普通多目标关联，输入仍是
 // 新的真实检测帧，不使用预测或任意同类目标替代。
 for(const auto&e:edges)if(e.score>=10&&tracks[e.t].id==selected){matches.push_back(e);used_tracks.insert(e.t);used_boxes.insert(e.b);}
 for(int stage=0;stage<3;stage++){
  std::vector<int>ts,bs;for(int t=0;t<(int)tracks.size();t++)if(!used_tracks.count(t)&&!tracks[t].retired){bool tentative=tracks[t].seen<cfg.confirm&&tracks[t].id!=selected;bool recent=f.time<=tracks[t].observed_at+(uint64_t)cfg.lost_ms*1000;if((stage==0&&!tentative)||(stage==1&&recent)||(stage==2&&tentative))ts.push_back(t);}
  for(int b=0;b<(int)boxes.size();b++)if(!used_boxes.count(b)&&((stage==1)==(boxes[b].score<.5f)))bs.push_back(b);
  if(ts.empty()||bs.empty())continue;double threshold=.58; // 各阶段使用相同综合证据条件，不设置IoU硬门限。
  // 每行增加一个空匹配项，其代价低于不满足条件的关联，避免强制匹配。
  
  std::vector<std::vector<double>>matrix(ts.size(),std::vector<double>(bs.size()+ts.size(),threshold+.0001));
  for(size_t r=0;r<ts.size();r++)for(size_t c=0;c<bs.size();c++){
   auto e=std::find_if(edges.begin(),edges.end(),[&](const Edge&e){return e.t==ts[r]&&e.b==bs[c];});matrix[r][c]=e==edges.end()?100.:e->evidence.cost;
  }
  auto assignment=byte_lap(matrix);for(size_t r=0;r<assignment.size();r++){int c=assignment[r];if(c<0||(size_t)c>=bs.size()||matrix[r][c]>threshold)continue;int t=ts[r],b=bs[(size_t)c];auto e=std::find_if(edges.begin(),edges.end(),[&](const Edge&e){return e.t==t&&e.b==b;});if(e!=edges.end()){matches.push_back(*e);used_tracks.insert(t);used_boxes.insert(b);}}
 }
 return matches;
}
