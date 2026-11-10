#include "relative_policy.hpp"
#include <cassert>

as2_test::OdomSample origin() {
    as2_test::OdomSample s; s.z=.3; s.mode=3; s.sequence=1; s.received=1; return s;
}
int main() {
    for (double sign : {-1.0, 1.0}) {
        auto s=origin(); s.yaw=1.5707963267948966;
        as2_test::RelativePolicy policy(s,1,false,sign*.5);
        bool reached=false;
        for(int i=1;i<70;++i) {
            s.sequence++; s.received=1+i*.05; s.y=sign*i*.01;
            const auto d=policy.update(s,s.received);
            assert(std::abs(d.vx)<=.15F && d.vyaw==0);
            if(d.stop) {assert(d.reached); reached=true; break;}
            assert(d.vx*sign>0);
        }
        assert(reached);
    }
    for (double sign : {-1.0, 1.0}) {
        auto s=origin(); s.yaw=sign*3.12;
        as2_test::RelativePolicy policy(s,1,true,sign*.5);
        bool reached=false;
        for(int i=1;i<80;++i) {
            s.sequence++; s.received=1+i*.05;
            s.yaw=std::atan2(std::sin(sign*(3.12+i*.01)),std::cos(sign*(3.12+i*.01)));
            const auto d=policy.update(s,s.received);
            assert(d.vx==0 && std::abs(d.vyaw)<=.25F);
            if(d.stop) {assert(d.reached); reached=true; break;}
            assert(d.vyaw*sign>0);
        }
        assert(reached);
    }
    {
        auto s=origin(); as2_test::RelativePolicy p(s,1,true,.5);
        s.sequence++; s.received=1.1; s.x=.08;
        assert(p.update(s,1.1).stop);
        s.x=0; s.sequence++; s.received=1.2;
        assert(p.update(s,1.2).stop);
    }
    {
        auto s=origin(); as2_test::RelativePolicy p(s,1,false,.5);
        assert(p.update(s,1.4).stop);
    }
    {
        auto s=origin(); as2_test::RelativePolicy p(s,1,false,.5);
        s.sequence++; s.received=3.1;
        const auto d=p.update(s,3.1); assert(d.stop && !d.reached);
    }
    for (double invalid : {0.0, 1.1, -3.0}) {
        bool rejected=false;
        try {as2_test::RelativePolicy p(origin(),1,false,invalid);} catch (const std::exception&) {rejected=true;}
        assert(rejected);
    }
}
