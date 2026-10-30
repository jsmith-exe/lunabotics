#ifndef GZ_TERRAMECHANICS_SYSTEM_HH_
#define GZ_TERRAMECHANICS_SYSTEM_HH_

#include<gz/sim/System.hh>
#include<memory>

namespace gz_terramechanics


{
  class TerramechanicsSystem:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemReset
  {

    public: 
      TerramechanicsSystem();
      ~TerramechanicsSystem() override;

      void Configure( 
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

      void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

      void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const  gz::sim::EntityComponentManager &_ecm) override;

      void Reset(const gz::sim::UpdateInfo &_info, 
      gz::sim::EntityComponentManager &_ecm) override;
  
    private:
      class TerramechanicsSystemPrivate;
      std::unique_ptr<TerramechanicsSystemPrivate> dataPtr;
    }; 

}

#endif



