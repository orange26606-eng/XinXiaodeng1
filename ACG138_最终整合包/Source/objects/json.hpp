#pragma once
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
namespace obj {
struct J { char type='n';double number=0;bool boolean=false;std::string text;std::map<std::string,J> fields;std::vector<J> values;
 const J&at(const std::string&k)const{auto i=fields.find(k);if(i==fields.end())throw std::runtime_error("missing JSON key "+k);return i->second;}
 std::string str()const{if(type!='s')throw std::runtime_error("JSON string required");return text;}
 double num()const{if(type!='d'||!std::isfinite(number))throw std::runtime_error("JSON number required");return number;}
};
inline uint64_t decimal(const std::string&s){if(s.empty()||s.size()>20)throw std::runtime_error("invalid decimal");uint64_t n=0;for(char c:s){if(c<'0'||c>'9'||n>(UINT64_MAX-(unsigned)(c-'0'))/10)throw std::runtime_error("decimal overflow");n=n*10+(unsigned)(c-'0');}return n;}
class JP {const std::string&s;size_t p=0,nodes=0;void ws(){while(p<s.size()&&(s[p]==' '||s[p]=='\n'||s[p]=='\r'||s[p]=='\t'))++p;}
 std::string string(){if(p>=s.size()||s[p++]!='"')throw std::runtime_error("string");std::string v;while(p<s.size()){char c=s[p++];if(c=='"')return v;if((unsigned char)c<32)throw std::runtime_error("control char");if(c=='\\'){if(p==s.size())break;c=s[p++];if(c=='n')c='\n';else if(c=='r')c='\r';else if(c=='t')c='\t';else if(c!='"'&&c!='\\'&&c!='/')throw std::runtime_error("unsupported escape");}v+=c;if(v.size()>4096)throw std::runtime_error("string limit");}throw std::runtime_error("unterminated string");}
 J value(int depth){ws();if(depth>12||++nodes>8192||p==s.size())throw std::runtime_error("JSON limit");J j;char c=s[p];if(c=='{'){j.type='o';++p;ws();if(p<s.size()&&s[p]=='}'){++p;return j;}for(;;){ws();auto k=string();ws();if(p==s.size()||s[p++]!=':')throw std::runtime_error("colon");auto v=value(depth+1);if(!j.fields.emplace(k,std::move(v)).second)throw std::runtime_error("duplicate key");ws();if(p==s.size())break;c=s[p++];if(c=='}')return j;if(c!=',')break;}throw std::runtime_error("object");}
 if(c=='['){j.type='a';++p;ws();if(p<s.size()&&s[p]==']'){++p;return j;}for(;;){j.values.push_back(value(depth+1));ws();if(p==s.size())break;c=s[p++];if(c==']')return j;if(c!=',')break;}throw std::runtime_error("array");}
 if(c=='"'){j.type='s';j.text=string();return j;}if(s.compare(p,4,"null")==0){p+=4;return j;}if(s.compare(p,4,"true")==0){p+=4;j.type='b';j.boolean=true;return j;}if(s.compare(p,5,"false")==0){p+=5;j.type='b';return j;}
 if(c!='-'&&(c<'0'||c>'9'))throw std::runtime_error("number");size_t a=p;if(s[p]=='-')++p;if(p==s.size())throw std::runtime_error("number");if(s[p]=='0')++p;else{if(s[p]<'1'||s[p]>'9')throw std::runtime_error("number");while(p<s.size()&&s[p]>='0'&&s[p]<='9')++p;}if(p<s.size()&&s[p]=='.'){++p;size_t q=p;while(p<s.size()&&s[p]>='0'&&s[p]<='9')++p;if(q==p)throw std::runtime_error("fraction");}if(p<s.size()&&(s[p]=='e'||s[p]=='E')){++p;if(p<s.size()&&(s[p]=='+'||s[p]=='-'))++p;size_t q=p;while(p<s.size()&&s[p]>='0'&&s[p]<='9')++p;if(q==p)throw std::runtime_error("exponent");}j.type='d';j.number=std::strtod(s.substr(a,p-a).c_str(),nullptr);if(!std::isfinite(j.number))throw std::runtime_error("nonfinite");return j;}
public:explicit JP(const std::string&t):s(t){}J parse(){J j=value(0);ws();if(p!=s.size())throw std::runtime_error("trailing JSON");return j;}};
inline std::string quote(const std::string&s){std::string o="\"";for(unsigned char c:s){if(c=='"'||c=='\\'){o+='\\';o+=(char)c;}else if(c=='\n')o+="\\n";else if(c=='\r')o+="\\r";else if(c=='\t')o+="\\t";else if(c<32)o+='?';else o+=(char)c;}return o+'"';}
}
