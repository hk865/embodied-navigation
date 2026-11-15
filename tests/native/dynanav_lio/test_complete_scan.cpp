// CPU synthetic-message tests; no simulator, external ROS services or robot.
#define main original_ouster_fixture_main
#include "test_ouster_mode.cpp"
#undef main

namespace {
void image(LIVMapper &mapper,double t) {
    auto msg=std::make_shared<sensor_msgs::msg::Image>();msg->header.stamp=rclcpp::Time(int64_t(std::llround(t*1e9)));
    msg->encoding="bgr8";msg->width=320;msg->height=240;msg->step=960;msg->data.resize(msg->height*msg->step,96);mapper.receive_image(msg);
}
void equal_state(const StatesGroup &a,const StatesGroup &b,const char *why) {
    require((a.rot_end-b.rot_end).norm()==0&&(a.pos_end-b.pos_end).norm()==0&&(a.vel_end-b.vel_end).norm()==0&&
        (a.bias_g-b.bias_g).norm()==0&&(a.bias_a-b.bias_a).norm()==0&&(a.gravity-b.gravity).norm()==0&&
        (a.cov-b.cov).norm()==0&&a.inv_expo_time==b.inv_expo_time,why);
}
using Scan = fastlivo_scan::Certificate;
Scan::SharedPtr scan(uint64_t s, uint64_t seq = 1, std::vector<uint32_t> times = {1525,25000000,50000000,75000000,99998448}) {
    auto out = std::make_shared<Scan>();
    out->contract_version = 1; out->session_id = "cpu-fixture-session";
    out->sequence = seq; out->native_frame_id = seq * 6;
    out->sensor_id = "go2_rtx_os1"; out->config_id = fastlivo_scan::config_id;
    out->source_start_ns = s; out->source_end_ns = s + fastlivo_scan::period_ns;
    out->coordinate_frame_start_ns=s+15000000ULL;out->coordinate_frame_end_ns=out->source_end_ns;
    out->accumulate_outputs = true; out->scan_complete = true;
    auto &m = out->cloud;
    m.header.stamp = rclcpp::Time(int64_t(s)); m.header.frame_id = "go2_imu";
    m.height=1; m.width=times.size(); m.point_step=32; m.row_step=32*m.width;
    m.is_bigendian=false; m.is_dense=true;
    const char *names[]={"x","y","z","intensity","t","reflectivity","ring","ambient","range"};
    const uint32_t offsets[]={0,4,8,12,16,20,22,24,28};
    const uint8_t types[]={7,7,7,7,6,4,2,4,6};
    for (int i=0;i<9;++i) { sensor_msgs::msg::PointField f; f.name=names[i];f.offset=offsets[i];f.datatype=types[i];f.count=1;m.fields.push_back(f); }
    m.data.resize(m.row_step);
    for(size_t i=0;i<times.size();++i) {
        const float p[]={2.f,0.f,0.f,float(i+100)};
        auto *d=m.data.data()+32*i;std::memcpy(d,p,16);std::memcpy(d+16,&times[i],4);d[22]=i%32;
    }
    return out;
}
Parameters enabled(bool on=true) { return {{"common.same_pair_vio_handoff",true},{"common.complete_scan_watermark",on}}; }
void feed(LIVMapper &m,const Scan::SharedPtr &c) {
    if(m.complete_scan_watermark) m.receive_complete_scan(c);
    else m.receive_cloud(std::make_shared<sensor_msgs::msg::PointCloud2>(c->cloud));
}
void same_points(const PointCloudXYZI &a,const PointCloudXYZI &b,const char *reason) {
    require(a.size()==b.size(),reason);
    for(size_t i=0;i<a.size();++i) require(a[i].x==b[i].x && a[i].y==b[i].y && a[i].z==b[i].z &&
        a[i].intensity==b[i].intensity && a[i].curvature==b[i].curvature,reason);
}
void check_contract() {
    for(int fault=0;fault<21;++fault) {
        auto a=scan(1000000000); fastlivo_scan::Watermark w; w.commit(*a,&a->cloud);
        auto b=scan(1100000000,2); const sensor_msgs::msg::PointCloud2 *bound=&b->cloud;
        auto foreign=b->cloud;
        switch(fault) {
            case 0:b->contract_version=2;break; case 1:b->scan_complete=false;break;
            case 2:b->accumulate_outputs=false;break; case 3:b->config_id="other";break;
            case 4:b->sensor_id="other";break; case 5:b->session_id="restart";break;
            case 6:b->sequence=3;break; case 7:b->native_frame_id=6;break;
            case 8:b->source_start_ns+=1; b->source_end_ns+=1;b->cloud.header.stamp=rclcpp::Time(int64_t(b->source_start_ns));break;
            case 9:b->source_end_ns++;break;case 10:b->cloud.header.stamp.nanosec++;break;
            case 11:bound=&foreign;break;case 12:b->cloud.data.pop_back();break;
            case 13:{uint32_t t=100000000;std::memcpy(b->cloud.data.data()+16,&t,4);break;}
            case 14:b->cloud.header.frame_id="wrong";break;case 15:b->cloud.is_bigendian=true;break;
            case 16:b->cloud.fields[4].datatype=7;break;case 17:b=scan(1000000000,1);bound=&b->cloud;break;
            case 18:{uint32_t t=1;std::memcpy(b->cloud.data.data()+48,&t,4);break;}
            case 19:b->coordinate_frame_start_ns++;break;
            case 20:b->coordinate_frame_end_ns++;break;
        }
        bool threw=false;try {w.commit(*b,bound);}catch(const std::runtime_error &){threw=true;}
        require(threw && w.failed && w.end_ns==1100000000 && w.sequence==1,"Invalid envelope crossed watermark");
        auto retry=scan(1100000000,2);threw=false;
        try{w.commit(*retry,&retry->cloud);}catch(const std::runtime_error&){threw=true;}
        require(threw && !w.covers(1.,1.1) && w.end_ns==1100000000,"Failed session was silently reseeded");
    }
    fastlivo_scan::Watermark w; auto a=scan(1000000000,7);w.commit(*a,&a->cloud);
    require(!w.covers(.99,1.1)&&w.covers(1.,1.1)&&!w.covers(1.,1.100000001),"Watermark invented earlier/later coverage");
    auto b=scan(1100000000,8,{});w.commit(*b,&b->cloud);
    require(w.end_ns==1200000000,"Empty complete interval was fabricated or discarded");
    auto n=node("complete_default");LIVMapper m(n,n->get_name());require(!m.complete_scan_watermark,"Opt-in changed default");
    std::cout<<"PASS 21 invalid certificates and permanent failure latch, exact interval, empty interval, default off\n";
}
void set_initialized(LIVMapper &m,double previous) {
    // Gate-only tests explicitly avoid claiming estimator equivalence.
    m.LidarMeasures.last_lio_update_time=previous;
    m.LidarMeasures.lio_vio_flg=WAIT;
}
void check_gates() {
    for(int mode=0;mode<9;++mode) {
        auto n=node("complete_gates"+std::to_string(mode),enabled(),true);LIVMapper m(n,n->get_name());
        const double target=10.1;set_initialized(m,(mode==4||mode==7)?9.99:10.);
        auto c=scan(10000000000);m.receive_complete_scan(c);
        image(m,mode==3?10.:target);
        m.receive_imu(imu(mode==2?10.09:target));
        if(mode==1 || mode==2 || mode==3 || mode==4 || mode==5) {
            *m.LidarMeasures.pcl_proc_next=*m.lid_raw_data_buffer.front().cloud;
            m.lid_raw_data_buffer.clear();m.lid_header_time_buffer.clear();
        }
        if(mode==8) {m.lid_raw_data_buffer.clear();m.lid_header_time_buffer.clear();m.LidarMeasures.pcl_proc_next->clear();}
        if(mode==5) m.LidarMeasures.pcl_proc_next->back().curvature=100.f;
        if(mode==6) {m.img_source_stamp_buffer.clear();bool threw=false;try{m.sync_packages(m.LidarMeasures);}catch(const std::runtime_error&){threw=true;}
            require(threw,"Missing original image identity accepted");continue;}
        const auto count=m.imu_buffer.size();
        const bool selected=m.sync_packages(m.LidarMeasures);
        if(mode==0 || mode==1) require(selected && m.LidarMeasures.pcl_proc_cur->size()==5,"Complete scan/pending failed to release");
        else require(!selected && (mode==3 || m.imu_buffer.size()==count),"Guard relaxed or pending boundary included");
    }
    auto n=node("complete_missing",enabled(),true);LIVMapper m(n,n->get_name());
    set_initialized(m,10.);
    for(double t:{10.,10.1}) {
        bool rejected=false;try{m.receive_cloud(cloud(t,{1525,99998448}));}catch(const std::runtime_error&){rejected=true;}
        require(rejected && m.lid_raw_data_buffer.empty() && !m.scan_watermark.valid,"Missing metadata accepted in typed-only mode");
    }
    bool rejected=false;try{m.standard_pcl_cbk(cloud(10.2,{1525,99998448}));}catch(const std::runtime_error&){rejected=true;}
    require(rejected,"Direct raw callback bypassed typed-only mode");
    auto coldn=node("complete_cold",enabled(),true);LIVMapper cold(coldn,coldn->get_name());
    cold.receive_complete_scan(scan(10000000000));image(cold,9.9);image(cold,10.1);cold.receive_imu(imu(10.1));
    require(cold.LidarMeasures.last_lio_update_time<0,"Cold fixture already initialized");
    require(!cold.sync_packages(cold.LidarMeasures)&&cold.LidarMeasures.last_lio_update_time==10.&&cold.img_buffer.size()==1,
            "Early camera frame poisoned certified startup origin");
    require(cold.sync_packages(cold.LidarMeasures)&&cold.LidarMeasures.pcl_proc_cur->size()==5,"Certified cold startup deadlocked");
    std::cout<<"PASS raw/pending exact end, IMU shortfall, old image, no pre-first coverage, pending exact boundary, image identity, absent metadata\n";
}
void check_ingress_failure() {
    for(int fault=0;fault<3;++fault) {
        auto n=node("complete_ingress_failure"+std::to_string(fault),enabled(),true);LIVMapper m(n,n->get_name());
        set_initialized(m,10.);m.receive_complete_scan(scan(10000000000));image(m,10.1);m.receive_imu(imu(10.1));
        auto bad=scan(10100000000,2);
        if(fault==0)bad->scan_complete=false;
        if(fault==1){bad=scan(10200000000,2);}
        if(fault==2){uint32_t t=100000000;std::memcpy(bad->cloud.data.data()+16,&t,4);}
        bool threw=false;try{m.receive_complete_scan(bad);}catch(const std::runtime_error&){threw=true;}
        require(threw&&m.scan_watermark.failed,"Actual malformed ingress failed to poison owner session");
        threw=false;try{m.receive_complete_scan(scan(10100000000,2));}catch(const std::runtime_error&){threw=true;}
        require(threw,"Actual failed ingress accepted later cloud");
        const auto count=m.imu_buffer.size();threw=false;try{m.sync_packages(m.LidarMeasures);}catch(const std::runtime_error&){threw=true;}
        require(threw&&m.imu_buffer.size()==count&&m.img_buffer.size()==1,"Failed ingress crossed sync using existing points");
    }
    std::cout<<"PASS actual ordered receive/decode/owner failure latch, no later cloud or sync bypass\n";
}
void initialize_scan_pair(LIVMapper &m) {
    feed(m,scan(10000000000));image(m,10.05);
    for(int i=1000;i<=1005;++i)m.receive_imu(imu(i/100.));
    require(m.sync_packages(m.LidarMeasures),"Init pair selection");
    m.handleFirstFrame();m.processImu();m.handleLIO();
    require(m.pending_pair.phase==PendingLivoPair::Skip,"Init falsely ready");
    require(!m.sync_packages(m.LidarMeasures),"Init falsely VIO");
}
void check_equivalence() {
    auto an=node("complete_equiv_old",enabled(false),true),bn=node("complete_equiv_new",enabled(),true);
    LIVMapper a(an,an->get_name()),b(bn,bn->get_name());
    image_transport::ImageTransport ai(an),bi(bn);a.initializeSubscribersAndPublishers(an,ai);b.initializeSubscribersAndPublishers(bn,bi);
    initialize_scan_pair(a);initialize_scan_pair(b);
    uint64_t next_old=2;
    for(uint64_t k=1;k<=4;++k) {
        const uint64_t target_ns=10000000000ULL+k*100000000ULL;const double target=fastlivo_scan::seconds(target_ns);
        if(k>1)feed(b,scan(target_ns-100000000,k));
        image(a,target);image(b,target);
        for(int i=1;i<=10;++i) {const double t=target-.1+i*.01;a.receive_imu(imu(t,.07));b.receive_imu(imu(t,.07));}
        require(!a.sync_packages(a.LidarMeasures),"Baseline unexpectedly stopped waiting");
        require(b.sync_packages(b.LidarMeasures),"Certified pending waited for future scan");
        feed(a,scan(target_ns,next_old++));
        require(a.sync_packages(a.LidarMeasures),"Baseline future tail did not release");
        same_points(*a.LidarMeasures.pcl_proc_cur,*b.LidarMeasures.pcl_proc_cur,"Current point order/float curvature differs");
        const auto am=a.LidarMeasures.measures.back(),bm=b.LidarMeasures.measures.back();
        require(am.lio_time==bm.lio_time&&am.imu.size()==bm.imu.size(),"Selected IMU target/list length differs");
        for(size_t i=0;i<am.imu.size();++i)require(*am.imu[i]==*bm.imu[i],"Selected IMU sample differs");
        a.processImu();b.processImu();equal_state(a._state,b._state,"Process2 state/cov differs");
        require(*a.p_imu->last_imu_sample()==*b.p_imu->last_imu_sample()&&a.p_imu->propagation_end_time()==b.p_imu->propagation_end_time(),"last_imu/prop_end differs");
        same_points(*a.feats_undistort,*b.feats_undistort,"Deskew points differ");
        a.handleLIO();b.handleLIO();equal_state(a._state,b._state,"LIO state/cov differs");
        require(a.pending_pair.phase==PendingLivoPair::LioReady&&b.pending_pair.phase==PendingLivoPair::LioReady,"Mature pair not ready");
        require(a.sync_packages(a.LidarMeasures)&&b.sync_packages(b.LidarMeasures),"VIO not selected");
        require(a.pending_pair.source_stamp_ns==target_ns&&b.pending_pair.source_stamp_ns==target_ns,"VIO source stamp differs");
        require(cv::norm(a.LidarMeasures.measures.back().img,b.LidarMeasures.measures.back().img)==0,"VIO image differs");
        a.processImu();b.processImu();a.handleVIO();b.handleVIO();equal_state(a._state,b._state,"VIO state/cov differs");
    }
    std::cout<<"PASS 4 real synthetic targets: current order/float curvature, full IMU list, deskew, last_imu, prop_end, state/cov, VIO image/source ns exact equality\n";
}
void check_recorded_fragment(const char *filename) {
    std::ifstream in(filename,std::ios::binary);
    const auto read=[&](void *p,size_t n){in.read(static_cast<char*>(p),n);require(bool(in),"Truncated recorded fixture");};
    uint64_t count;read(&count,8);require(count==12,"Expected 12 original scans");
    std::vector<Scan::SharedPtr> scans;
    for(uint64_t i=0;i<count;++i) {
        uint64_t start,bytes;read(&start,8);read(&bytes,8);require(bytes<=2*1024*1024 && bytes%32==0,"Bad original cloud bytes");
        auto c=scan(start,i+1,{});c->cloud.width=bytes/32;c->cloud.row_step=bytes;c->cloud.data.resize(bytes);read(c->cloud.data.data(),bytes);scans.push_back(c);
    }
    read(&count,8);std::vector<sensor_msgs::msg::Imu::SharedPtr> imus;
    for(uint64_t i=0;i<count;++i) {
        uint64_t ns;double v[37];read(&ns,8);read(v,sizeof(v));auto m=imu(0);m->header.stamp=rclcpp::Time(int64_t(ns));
        m->header.frame_id="go2_imu";
        m->angular_velocity.x=v[0];m->angular_velocity.y=v[1];m->angular_velocity.z=v[2];
        m->linear_acceleration.x=v[3];m->linear_acceleration.y=v[4];m->linear_acceleration.z=v[5];
        m->orientation.x=v[6];m->orientation.y=v[7];m->orientation.z=v[8];m->orientation.w=v[9];
        std::copy(v+10,v+19,m->orientation_covariance.begin());std::copy(v+19,v+28,m->angular_velocity_covariance.begin());
        std::copy(v+28,v+37,m->linear_acceleration_covariance.begin());imus.push_back(m);
    }
    auto an=node("fragment_old",enabled(false),true),bn=node("fragment_new",enabled(),true);
    LIVMapper a(an,an->get_name()),b(bn,bn->get_name());image_transport::ImageTransport ai(an),bi(bn);
    a.initializeSubscribersAndPublishers(an,ai);b.initializeSubscribersAndPublishers(bn,bi);
    size_t next_imu=0;
    const auto feed_imu=[&](uint64_t end){while(next_imu<imus.size() && uint64_t(rclcpp::Time(imus[next_imu]->header.stamp).nanoseconds())<=end) {
        a.receive_imu(imus[next_imu]);b.receive_imu(imus[next_imu]);++next_imu;}};
    const uint64_t start=scans.front()->source_start_ns;
    feed(a,scans[0]);feed(b,scans[0]);image(a,fastlivo_scan::seconds(start+50000000));image(b,fastlivo_scan::seconds(start+50000000));feed_imu(start+55000000);
    require(a.sync_packages(a.LidarMeasures)&&b.sync_packages(b.LidarMeasures),"Recorded fragment init selection failed");
    a.handleFirstFrame();b.handleFirstFrame();a.processImu();b.processImu();a.handleLIO();b.handleLIO();
    require(!a.sync_packages(a.LidarMeasures)&&!b.sync_packages(b.LidarMeasures),"Recorded init skip failed");
    for(size_t k=1;k<scans.size();++k) {
        const uint64_t target_ns=scans[k]->source_start_ns;const double target=fastlivo_scan::seconds(target_ns);
        if(k>1)feed(b,scans[k-1]);image(a,target);image(b,target);feed_imu(target_ns+5000000);
        require(!a.sync_packages(a.LidarMeasures),"Recorded baseline should await next scan");
        require(b.sync_packages(b.LidarMeasures),"Recorded certified interval did not release");
        feed(a,scans[k]);require(a.sync_packages(a.LidarMeasures),"Recorded baseline tail release failed");
        same_points(*a.LidarMeasures.pcl_proc_cur,*b.LidarMeasures.pcl_proc_cur,"Recorded current order/curvature float mismatch");
        const auto am=a.LidarMeasures.measures.back(),bm=b.LidarMeasures.measures.back();
        require(am.lio_time==bm.lio_time&&am.imu.size()==bm.imu.size(),"Recorded IMU target/count mismatch");
        for(size_t i=0;i<am.imu.size();++i)require(*am.imu[i]==*bm.imu[i],"Recorded full IMU list mismatch");
        a.processImu();b.processImu();equal_state(a._state,b._state,"Recorded Process2 state/cov mismatch");
        same_points(*a.feats_undistort,*b.feats_undistort,"Recorded deskew mismatch");
        require(*a.p_imu->last_imu_sample()==*b.p_imu->last_imu_sample()&&a.p_imu->propagation_end_time()==b.p_imu->propagation_end_time(),"Recorded last_imu/prop_end mismatch");
        a.handleLIO();b.handleLIO();equal_state(a._state,b._state,"Recorded LIO posterior/cov mismatch");
        require(a.sync_packages(a.LidarMeasures)&&b.sync_packages(b.LidarMeasures),"Recorded VIO selection failed");
        require(a.pending_pair.source_stamp_ns==target_ns&&b.pending_pair.source_stamp_ns==target_ns,"Recorded source identity mismatch");
        require(cv::norm(a.LidarMeasures.measures.back().img,b.LidarMeasures.measures.back().img)==0,"Recorded fixture VIO input differs");
        a.processImu();b.processImu();a.handleVIO();b.handleVIO();equal_state(a._state,b._state,"Recorded fixture VIO state/cov mismatch");
        std::cout<<"FRAGMENT_EQ source_ns="<<target_ns<<" points="<<a.LidarMeasures.pcl_proc_cur->size()<<" imu="<<am.imu.size()<<" exact=1\n";
    }
    std::cout<<"PASS 11 targets from 12 SHA-verified original scans and recorded IMU; same fresh initial state/blank images, NOT original trajectory/map/VIO replay\n";
}
void check_empty() {
    for(int allfiltered=0;allfiltered<2;++allfiltered) {
        auto n=node("complete_empty"+std::to_string(allfiltered),enabled(),true);LIVMapper m(n,n->get_name());
        image_transport::ImageTransport it(n);m.initializeSubscribersAndPublishers(n,it);
        initialize_scan_pair(m);
        m.LidarMeasures.pcl_proc_next->clear();
        auto c=scan(10100000000,2,allfiltered?std::vector<uint32_t>{1525,99998448}:std::vector<uint32_t>{});
        if(allfiltered)for(size_t i=0;i<c->cloud.width;++i){float zero=0;std::memcpy(c->cloud.data.data()+32*i,&zero,4);}
        feed(m,c);image(m,10.2);for(int i=1006;i<=1020;++i)m.receive_imu(imu(i/100.));
        require(m.sync_packages(m.LidarMeasures)&&m.LidarMeasures.pcl_proc_cur->empty(),"Empty interval not selected empty");
        m.processImu();m.handleLIO();require(m.pending_pair.phase==PendingLivoPair::Skip&&m.pending_pair.outcome==PendingLivoPair::EmptyCloud,"Empty interval falsely LioReady");
        require(!m.sync_packages(m.LidarMeasures)&&m.img_buffer.empty(),"Empty skip falsely VIO or image retained");
        require(m.LidarMeasures.last_lio_update_time==10.2&&m.p_imu->propagation_end_time()==10.2,"Empty skip lost real IMU propagation endpoint");
        feed(m,scan(10200000000,3));image(m,10.3);for(int i=1021;i<=1030;++i)m.receive_imu(imu(i/100.));
        require(m.sync_packages(m.LidarMeasures)&&m.LidarMeasures.pcl_proc_cur->size()==5,"Valid scan after empty skip blocked/reused points");
        m.processImu();m.handleLIO();require(m.pending_pair.phase==PendingLivoPair::LioReady&&m.p_imu->propagation_end_time()==10.3,
            "Valid scan after empty skip failed IMU/LIO lifecycle");
        require(m.sync_packages(m.LidarMeasures),"Valid scan after empty skip lost VIO");m.processImu();m.handleVIO();
    }
    std::cout<<"PASS zero return/all filtered remains EmptyCloud/Skip, no VIO or fake points\n";
}
}
int main(int argc,char **argv) {
    rclcpp::init(argc,argv);
    try {check_contract();check_gates();check_ingress_failure();check_equivalence();check_empty();if(const char *p=std::getenv("COMPLETE_SCAN_REPLAY_BIN"))check_recorded_fragment(p);rclcpp::shutdown();return 0;}
    catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<'\n';rclcpp::shutdown();return 1;}
}
