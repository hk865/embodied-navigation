#include "a_star/a_star_search.h"
#include <fstream>
#include <iomanip>
int main(int argc,char**argv){
 if(argc!=4)return 2;
 std::ifstream f(argv[1],std::ios::binary);int32_t dims[3];f.read((char*)dims,sizeof(dims));int L=dims[0],H=dims[1],W=dims[2];double res;f.read((char*)&res,sizeof(res));
 Eigen::MatrixXd cost(L*H,W),height(L*H,W),gateway(L*H,W);
 for(auto m:{&cost,&height,&gateway})for(int i=0;i<L*H;i++)for(int j=0;j<W;j++){double v;f.read((char*)&v,8);(*m)(i,j)=v;}
 if(!f)return 3;
 std::ifstream in(argv[2]);Eigen::Vector3i start,goal;in>>start[0]>>start[1]>>start[2]>>goal[0]>>goal[1]>>goal[2];
 Astar astar;astar.Init(20,L,res,1,cost,height,gateway);bool found=astar.Search(start,goal);auto path=astar.GetResultMatrix();std::ofstream out(argv[3]);out<<std::setprecision(16)<<"{\"found\":"<<(found?"true":"false")<<",\"cells_layer_row_col\":[";
 for(int i=0;i<path.rows();i++){if(i)out<<',';out<<'['<<path(i,0)<<','<<path(i,1)<<','<<path(i,2)<<']';}out<<"]}";return found?0:4;
}
