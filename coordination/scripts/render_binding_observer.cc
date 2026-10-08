// Opt-in, read-only development evidence. No transport publishers or controls.
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <cstdint>
#include <map>
#include <mutex>

namespace gazebo {
class RenderBindingObserver : public SystemPlugin {
  event::ConnectionPtr connection;
  std::map<std::string, event::ConnectionPtr> imageConnections;
  uint64_t seq = 0;
  std::mutex writeMutex;
  static void Pose(std::ostream &out, const ignition::math::Pose3d &p) {
    out << "{\"xyz\":[" << p.Pos().X() << ',' << p.Pos().Y() << ',' << p.Pos().Z()
        << "],\"quaternion_xyzw\":[" << p.Rot().X() << ',' << p.Rot().Y() << ','
        << p.Rot().Z() << ',' << p.Rot().W() << "]}";
  }
 public:
  void Load(int, char **) override {
    connection = event::Events::ConnectPostRender([this]() { Observe(); });
  }
  void Observe() {
    if (!std::getenv("RENDER_BINDING_OUTPUT")) return;
    auto scene = rendering::get_scene("default");
    if (!scene) return;
    for (uint32_t i=0; i<scene->CameraCount(); ++i) {
        auto cam = scene->GetCamera(i);
        if (!cam || cam->ScopedName().find("cgo3_camera") == std::string::npos) continue;
        if (imageConnections.count(cam->ScopedName())) continue;
        imageConnections[cam->ScopedName()] = cam->ConnectNewImageFrame(
          [this, cam](const unsigned char *image, unsigned w, unsigned h,
                      unsigned depth, const std::string &format) {
            WriteFrame(cam, image, w, h, depth, format);
          });
    }
  }
  void WriteFrame(rendering::CameraPtr cam, const unsigned char *image,
                  unsigned w, unsigned h, unsigned depth, const std::string &format) {
    const char *path = std::getenv("RENDER_BINDING_OUTPUT");
    auto scene = cam->GetScene();
    if (!path || !scene || !image || !w || !h || depth != 3) return;
    std::lock_guard<std::mutex> guard(writeMutex);
    try {
        const double wall = cam->LastRenderWallTime().Double();
        const unsigned bytes = w*h*depth;
        uint64_t hash = 14695981039346656037ULL;
        for (unsigned k=0; k<bytes; ++k) {
          hash ^= image[k]; hash *= 1099511628211ULL;
        }
        std::ofstream out(path, std::ios::app);
        out << std::setprecision(17) << "{\"schema_version\":2,\"hook\":\"NewImageFrame\",\"control_input\":false,\"seq\":"
            << ++seq << ",\"scene_s\":" << scene->SimTime().Double()
            << ",\"render_wall_s\":" << wall << ",\"camera\":" << std::quoted(cam->ScopedName())
            << ",\"format\":" << std::quoted(format)
            << ",\"size\":[" << w << ',' << h << ',' << depth
            << "],\"image_bytes\":" << bytes
            << ",\"fnv1a64\":" << std::quoted(std::to_string(hash)) << ",\"camera_pose\":";
        Pose(out, cam->WorldPose());
        out << ",\"visuals\":[";
        bool comma = false;
        for (unsigned a=0; a<6; ++a) {
          const std::string actor = "actor_" + std::to_string(a);
          for (const auto &name : {actor, actor+"::"+actor+"_pose", actor+"::"+actor+"_pose::"+actor+"_visual"}) {
            auto visual = scene->GetVisual(name);
            if (!visual) continue;
            if (comma) out << ','; comma = true;
            out << "{\"name\":" << std::quoted(name) << ",\"visible\":"
                << (visual->GetVisible() ? "true" : "false") << ",\"world_pose\":";
            Pose(out, visual->WorldPose());
            out << ",\"bones\":[";
            auto node = visual->GetSceneNode(); bool boneComma = false;
            for (unsigned j=0; node && j<node->numAttachedObjects(); ++j) {
              auto entity = dynamic_cast<Ogre::Entity *>(node->getAttachedObject(j));
              if (!entity || !entity->hasSkeleton()) continue;
              auto skeleton = entity->getSkeleton();
              for (unsigned k=0; k<skeleton->getNumBones(); ++k) {
                auto bone = skeleton->getBone(k);
                if (bone->getName() != "Hips" && bone->getName() != "LeftFoot" && bone->getName() != "RightFoot") continue;
                auto derived = bone->_getDerivedPosition();
                auto world = node->_getFullTransform() * derived;
                if (boneComma) out << ','; boneComma = true;
                out << "{\"name\":" << std::quoted(bone->getName()) << ",\"derived\":["
                    << derived.x << ',' << derived.y << ',' << derived.z << "],\"node_world\":["
                    << world.x << ',' << world.y << ',' << world.z << "]}";
              }
            }
            out << "]}";
          }
        }
        out << "]}\n";
    } catch (const std::exception &) {
      // A missing diagnostic must never stop physics or issue flight commands.
    }
  }
};
GZ_REGISTER_SYSTEM_PLUGIN(RenderBindingObserver)
}
