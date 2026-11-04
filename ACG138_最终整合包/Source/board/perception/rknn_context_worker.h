#pragma once
// Api、ns()和check()由运行程序或无设备测试提供。
static inline void model_api_ok(int rc,const char*operation){if(rc){fprintf(stderr,"MODEL_API_ERROR operation=%s return_code=%d\n",operation,rc);throw std::runtime_error(std::string(operation)+" return_code="+std::to_string(rc));}}
struct InferenceTiming{uint64_t preprocess=0,input=0,run=0,get=0,decode=0,release=0;};
struct Context {
 Api&api;rknn_context ctx=0;ModelProfile profile=ModelProfile::load();unsigned output_count=9;bool decoded_transposed=false;std::array<rknn_tensor_attr,9>attrs{};InferenceTiming timing;
 std::vector<uint8_t>input=std::vector<uint8_t>(640*640*3);InputPlan plan;
 Context(Api&a,const std::vector<char>&model,unsigned worker):api(a){
  check(worker<3,"NPU worker index must be0..2");model_api_ok(api.init(&ctx,const_cast<char*>(model.data()),(uint32_t)model.size(),0,nullptr),"rknn_init");
  try{
   rknn_sdk_version version{};check(api.query(ctx,RKNN_QUERY_SDK_VERSION,&version,sizeof(version))==0,"RKNN version query failed");
   fprintf(stderr,"MODEL_LOAD profile=%s bytes=%zu source_sha256=%s worker=%u runtime=%s driver=%s core_mask=%u\n",profile.name.c_str(),model.size(),std::getenv("ACG_MODEL_SHA256")?std::getenv("ACG_MODEL_SHA256"):"NOT_PROVIDED",worker,version.api_version,version.drv_version,1u<<worker);
   check(api.core(ctx,(rknn_core_mask)(1u<<worker))==0,"NPU core binding failed");
   rknn_input_output_num count{};int query_code=api.query(ctx,RKNN_QUERY_IN_OUT_NUM,&count,sizeof(count));fprintf(stderr,"MODEL_IO profile=%s query_rc=%d actual_inputs=%u actual_outputs=%u expected_inputs=1 allowed_outputs=1or9\n",profile.name.c_str(),query_code,count.n_input,count.n_output);model_api_ok(query_code,"rknn_query_IO");check(count.n_input==1&&(count.n_output==1||count.n_output==9),"MODEL_OUTPUT_COUNT expected1or9");output_count=count.n_output;
   check((profile.name=="custom6"&&output_count==1)||(profile.name=="custom6-new")||(profile.name=="coco80"&&output_count==9),"MODEL_PROFILE_OUTPUT_COUNT_MISMATCH");
   rknn_tensor_attr in{};check(api.query(ctx,RKNN_QUERY_INPUT_ATTR,&in,sizeof(in))==0,"Input query failed");bool nchw=in.fmt==RKNN_TENSOR_NCHW;
   fprintf(stderr,"MODEL_INPUT worker=%u dims=%u,%u,%u,%u fmt=%d type=%d qnt=%d zp=%d scale=%.9g mean=0 std=255 RGB_uint8 once\n",worker,in.dims[0],in.dims[1],in.dims[2],in.dims[3],in.fmt,in.type,in.qnt_type,in.zp,in.scale);
   check(in.n_dims==4&&in.dims[0]==1&&(in.fmt==RKNN_TENSOR_NCHW||in.fmt==RKNN_TENSOR_NHWC)&&(nchw?(in.dims[1]==3&&in.dims[2]==640&&in.dims[3]==640):(in.dims[1]==640&&in.dims[2]==640&&in.dims[3]==3)),"MODEL_INPUT_SHAPE expectedRGB640x640");

   unsigned roles[3]{};
   for(unsigned i=0;i<output_count;i++){
    auto&t=attrs[i];t.index=i;check(api.query(ctx,RKNN_QUERY_OUTPUT_ATTR,&t,sizeof(t))==0,"Output query failed");
    fprintf(stderr,"MODEL_OUTPUT worker=%u index=%u name=%s ndim=%u dims=%u,%u,%u,%u fmt=%d type=%d qnt=%d zp=%d scale=%.9g elements=%u bytes=%u\n",worker,i,t.name,t.n_dims,t.dims[0],t.dims[1],t.dims[2],t.dims[3],t.fmt,t.type,t.qnt_type,t.zp,t.scale,t.n_elems,t.size);
    if(output_count==1){check(t.n_dims==3&&t.dims[0]==1&&((t.dims[1]==10&&t.dims[2]==8400)||(t.dims[1]==8400&&t.dims[2]==10)),"CUSTOM6_OUTPUT_SHAPE expected[1,10,8400]");decoded_transposed=t.dims[1]==8400;}
    else{
     check(t.n_dims==4&&t.dims[0]==1&&(t.fmt==RKNN_TENSOR_NCHW||t.fmt==RKNN_TENSOR_NHWC),"MODEL_HEAD_LAYOUT expectedNCHWorNHWC");bool nhwc=t.fmt==RKNN_TENSOR_NHWC;unsigned c=t.dims[nhwc?3:1],h=t.dims[nhwc?1:2],w=t.dims[nhwc?2:3];
     check(h==w&&(w==80||w==40||w==20),"MODEL_HEAD_GRID expected80or40or20");unsigned g=w==80?0:w==40?1:2,role=c==64?1:c==profile.labels.size()?2:c==1?4:0;
     check(role&&!(roles[g]&role),"MODEL_HEAD_CLASS_OR_DUPLICATE_ROLE_MISMATCH");roles[g]|=role;
    }
   }
   if(output_count==9)for(unsigned bits:roles)check(bits==7,"MODEL_HEAD_MISSING_REG_CLASS_OR_SUM");
  }catch(...){api.destroy(ctx);ctx=0;throw;}
 }
 ~Context(){if(ctx)api.destroy(ctx);}
 std::vector<Box>infer(const uint8_t*nv12,int eye){
  check(nv12&&eye>=0&&eye<=1,"INFERENCE_SOURCE_INVALID");uint64_t s=ns();plan.convert(nv12,eye,input.data());uint64_t end=ns();timing.preprocess=end-s;s=end;
  rknn_input in{};in.index=0;in.buf=input.data();in.size=(uint32_t)input.size();in.type=RKNN_TENSOR_UINT8;in.fmt=RKNN_TENSOR_NHWC;in.pass_through=0;
  model_api_ok(api.inputs(ctx,1,&in),"rknn_inputs_set");end=ns();timing.input=end-s;s=end;model_api_ok(api.run(ctx,nullptr),"rknn_run");end=ns();timing.run=end-s;s=end;
  std::array<rknn_output,9>outputs{};for(unsigned i=0;i<output_count;i++){outputs[i].index=i;outputs[i].want_float=1;}
  model_api_ok(api.get(ctx,output_count,outputs.data(),nullptr),"rknn_outputs_get");end=ns();timing.get=end-s;s=end;bool held=true;
  auto release=[&]{held=false;int rc=api.release(ctx,output_count,outputs.data());model_api_ok(rc,"rknn_outputs_release");};
  try{
   std::vector<Box>boxes;
   if(output_count==1){check(outputs[0].buf&&outputs[0].size>=10*8400*sizeof(float),"CUSTOM6_OUTPUT_TOO_SHORT expected336000_float_bytes");boxes=decode_decoded(static_cast<float*>(outputs[0].buf),outputs[0].size/sizeof(float),10,8400,decoded_transposed,eye);}
   else{
    std::array<Tensor,9>tensors{};
    for(unsigned i=0;i<9;i++){const auto&a=attrs[i];bool nhwc=a.fmt==RKNN_TENSOR_NHWC;unsigned c=a.dims[nhwc?3:1],h=a.dims[nhwc?1:2],w=a.dims[nhwc?2:3];check(outputs[i].buf&&outputs[i].size>=c*h*w*sizeof(float),"MODEL_HEAD_FLOAT_BUFFER_TOO_SHORT");tensors[i]={static_cast<float*>(outputs[i].buf),c,h,w,nhwc};}
    boxes=decode(tensors,eye,profile.name=="coco80"?.25f:.10f,(unsigned)profile.labels.size(),profile.name=="coco80");
   }
   end=ns();timing.decode=end-s;s=end;release();timing.release=ns()-s;return boxes;
  }catch(...){if(held){held=false;api.release(ctx,output_count,outputs.data());}throw;}
 }
};
