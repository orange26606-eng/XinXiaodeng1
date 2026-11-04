#pragma once
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
namespace acg {
struct ModelProfile {
 std::string name="coco80";std::vector<std::string>labels;
 static ModelProfile load(std::string requested="",std::string classes="",bool environment=true){
  if(environment)if(const char*p=std::getenv("ACG_YOLO_PROFILE"))if(*p)requested=p;
  if(environment)if(const char*p=std::getenv("ACG_OBJECT_CLASSES_FILE"))if(*p)classes=p;
  if(requested.empty())requested="coco80";ModelProfile out;out.name=requested;
  if((requested=="custom6"||requested=="custom6-new"))out.labels={"person","cup","cell_phone","bottle","box","remote"};
  else if(requested=="coco80")out.labels={"person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed","dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"};
  else throw std::runtime_error("MODEL_PROFILE_UNKNOWN");
  if(!classes.empty()){
   std::ifstream file(classes);if(!file)throw std::runtime_error("MODEL_CLASSES_FILE_MISSING");std::vector<std::string>names;std::string line;
   while(std::getline(file,line)){if(!line.empty()&&line.back()=='\r')line.pop_back();if(line.empty()||line[0]=='#')continue;if(line.size()>96)throw std::runtime_error("MODEL_CLASS_LABEL_TOO_LONG");for(unsigned char c:line)if(c<32||c=='"'||c=='\\')throw std::runtime_error("MODEL_CLASS_LABEL_INVALID");names.push_back(line);if(names.size()>80)throw std::runtime_error("MODEL_CLASS_LIMIT");}
   if(names.size()!=out.labels.size())throw std::runtime_error("MODEL_CLASSES_PROFILE_COUNT_MISMATCH");out.labels=std::move(names);
  }
  return out;
 }
 const char*label(int id)const{return id==80?"manual target":id>=0&&(size_t)id<labels.size()?labels[(size_t)id].c_str():"unknown";}
};
}
