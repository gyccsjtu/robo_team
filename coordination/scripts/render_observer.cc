// Development observer only: never supplies perception or flight inputs.
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/rendering.hh>
#include <fstream>
#include <chrono>
#include <cstdlib>

namespace gazebo {
class RenderObserver : public SystemPlugin {
  event::ConnectionPtr connection;
  std::chrono::steady_clock::time_point last;
 public:
  void Load(int, char **) override {
    connection = event::Events::ConnectPostRender([this]() { Observe(); });
  }
  void Observe() {
    auto now = std::chrono::steady_clock::now();
    if (now-last < std::chrono::seconds(1)) return;
    last = now;
    const char *path = std::getenv("RENDER_OBSERVER_OUTPUT");
    if (!path) return;
    auto scene = rendering::get_scene("default");
    if (!scene) return;
    std::ofstream out(path, std::ios::app);
    for (const auto &name : {"actor_0", "actor_0::actor_0_pose",
         "actor_0::actor_0_pose::actor_0_visual", "uav_1::cgo3_camera_link"}) {
      auto visual = scene->GetVisual(name);
      out << name << " found=" << bool(visual);
      if (visual) out << " local=" << visual->Pose() << " world=" << visual->WorldPose()
                      << " id=" << visual->GetId() << " visible=" << visual->GetVisible()
                      << " bounds=" << visual->BoundingBox();
      out << '\n';
      if (visual) {
        auto node = visual->GetSceneNode();
        out << " node objects=" << node->numAttachedObjects()
            << " transparency=" << visual->GetTransparency() << '\n';
        for (unsigned i=0; i<node->numAttachedObjects(); ++i) {
          auto object = node->getAttachedObject(i);
          out << " object=" << object->getName() << " visible=" << object->isVisible()
              << " flags=" << object->getVisibilityFlags() << " bounds=" << object->getBoundingBox() << '\n';
          auto entity = dynamic_cast<Ogre::Entity *>(object);
          if (entity) {
            auto mesh = entity->getMesh();
            out << " mesh=" << mesh->getName() << " submeshes=" << mesh->getNumSubMeshes()
                << " skeleton=" << entity->hasSkeleton() << '\n';
            for (unsigned j=0; j<mesh->getNumSubMeshes(); ++j) {
              auto sub = mesh->getSubMesh(j);
              auto vertices = sub->useSharedVertices ? mesh->sharedVertexData : sub->vertexData;
              out << " vertices=" << (vertices ? vertices->vertexCount : 0)
                  << " indices=" << sub->indexData->indexCount
                  << " material=" << sub->getMaterialName() << '\n';
              if (vertices) {
                auto element = vertices->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
                auto buffer = vertices->vertexBufferBinding->getBuffer(element->getSource());
                auto bytes = static_cast<unsigned char *>(buffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
                Ogre::AxisAlignedBox bounds;
                for (size_t k=0; k<vertices->vertexCount; ++k) {
                  float *p;
                  element->baseVertexPointerToElement(bytes+(vertices->vertexStart+k)*buffer->getVertexSize(), &p);
                  bounds.merge(Ogre::Vector3(p[0],p[1],p[2]));
                }
                buffer->unlock();
                out << " base vertices bounds=" << bounds << '\n';
              }
            }
            if (entity->hasSkeleton()) {
              auto skeleton = entity->getSkeleton();
              out << " bones=" << skeleton->getNumBones() << '\n';
              for (unsigned j=0; j<std::min<unsigned>(3, skeleton->getNumBones()); ++j) {
                auto bone = skeleton->getBone(j);
                out << " bone=" << bone->getName() << " position=" << bone->getPosition()
                    << " derived=" << bone->_getDerivedPosition() << '\n';
              }
            }
          }
        }
      }
    }
  }
};
GZ_REGISTER_SYSTEM_PLUGIN(RenderObserver)
}
