#ifndef ATLAS_CONVERTER_HPP_
#define ATLAS_CONVERTER_HPP


#include "Converter.hpp"


namespace Converter {

  class AtlasConverter {
    using map_point = orbslam3_interfaces::msg::MapPoint;
    using map = orbslam3_interfaces::msg::Map;
    using atlas = orbslam3_interfaces::msg::Atlas;

    using orb_map_point = ORB_SLAM3::MapPoint;
    using orb_map = ORB_SLAM3::Map;
    using orb_keyframe = ORB_SLAM3::KeyFrame;
    using orb_atlas = ORB_SLAM3::Atlas;

    
    //static atlas OrbAtlasToRosAtlas(orb_atlas* mpAtlas) {
    //    //std::mutex mMutexNewMP;
    //    //std::lock_guard<std::mutex> lock(mMutexNewMP);
    //    
    //    atlas rAtlas;
    //    rAtlas.mp_current_map = Converter::MapConverter::OrbMapToRosMap(mpAtlas->GetCurrentMap());
    //    return rAtlas;
    //}

  };
};

#endif
