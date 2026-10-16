#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "Modules/ModuleManager.h"
#include "UBotExtensionRegistry.h"
#include "UBotPackageRegistry.h"
#include "UBotPhysics.h"
#include "UBotPhysicsBackend.h"
#include "UBotPhysicsLibrary.h"
#include "UBotPhysicsTestTypes.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsBackendsExtensionPointTest, "UBotPhysics.Backends.ExtensionPoint", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsBackendsExtensionPointTest::RunTest(const FString& Parameters)
{
    using namespace UBotPhysicsTests;

    FUBotExtensionRegistry& Registry = FUBotExtensionRegistry::Get();
    TestTrue(TEXT("The UBotPhysics module is loaded"), FModuleManager::Get().IsModuleLoaded(TEXT("UBotPhysics")));
    TestTrue(TEXT("The module declares the backend extension point"), Registry.IsExtensionPointDeclared(UBot::Physics::BackendExtensionPoint));

    const TArray<FUBotExtensionPointInfo> Points = Registry.GetExtensionPoints();
    const FUBotExtensionPointInfo* Point = Points.FindByPredicate([](const FUBotExtensionPointInfo& Info)
    {
        return Info.Name == UBot::Physics::BackendExtensionPoint;
    });
    if (TestNotNull(TEXT("The point is listed"), Point))
    {
        TestEqual(TEXT("The point is owned by the package"), Point->OwnerPackage, UBot::Physics::PackageName);
        TestTrue(TEXT("The description says the point is experimental"), Point->Description.ToString().Contains(TEXT("Experimental")));
    }

    TestFalse(TEXT("No test backend before it is registered"), UBot::Physics::FindBackendFactory(BackendName).IsValid());
    TestFalse(TEXT("It is not listed either"), UBot::Physics::GetBackendNames().Contains(BackendName));
    TestFalse(TEXT("A backend with no name is never found"), UBot::Physics::FindBackendFactory(NAME_None).IsValid());

    {
        FScopedTestBackend Backend;
        TestTrue(TEXT("The test backend registers"), Backend.bRegistered);

        const TSharedPtr<IUBotPhysicsBackendFactory> Found = UBot::Physics::FindBackendFactory(BackendName);
        if (TestTrue(TEXT("The test backend is found by name"), Found.IsValid()))
        {
            TestTrue(TEXT("It is the registered factory"), Found.Get() == static_cast<IUBotPhysicsBackendFactory*>(&Backend.Factory.Get()));
        }
        TestTrue(TEXT("The test backend is listed"), UBot::Physics::GetBackendNames().Contains(BackendName));
        TestTrue(TEXT("The Blueprint library lists it too"), UUBotPhysicsLibrary::GetPhysicsBackendNames().Contains(BackendName));
        TestFalse(TEXT("Another name is not found"), UBot::Physics::FindBackendFactory(TEXT("NoSuchBackend")).IsValid());

        FUBotPhysicsCapabilities Capabilities;
        TestTrue(TEXT("The capabilities of a registered backend are reported"), UUBotPhysicsLibrary::GetPhysicsBackendCapabilities(BackendName, Capabilities));
        TestTrue(TEXT("Rigid bodies"), Capabilities.bRigidBodies);
        TestTrue(TEXT("Kinematic foreign bodies"), Capabilities.bKinematicForeignBodies);
        TestTrue(TEXT("Deterministic"), Capabilities.bDeterministic);
        TestFalse(TEXT("Articulations are not claimed"), Capabilities.bArticulations);
        TestFalse(TEXT("Soft bodies are not claimed"), Capabilities.bSoftBodies);
        TestFalse(TEXT("Accelerations are not claimed"), Capabilities.bAccelerations);

        const FName OtherPackage(TEXT("UBotPhysicsOtherTests"));
        TestFalse(TEXT("A second registration under the same name is refused"),
            Registry.RegisterExtension(UBot::Physics::BackendExtensionPoint, OtherPackage, MakeShared<FUBotPhysicsTestBackendFactory>()));
        Registry.UnregisterAllFromPackage(OtherPackage);

        FUBotPhysicsCapabilities Unknown;
        Unknown.bRigidBodies = true;
        TestFalse(TEXT("The capabilities of an unknown backend are not reported"), UUBotPhysicsLibrary::GetPhysicsBackendCapabilities(TEXT("NoSuchBackend"), Unknown));
        TestFalse(TEXT("And come back empty"), Unknown.bRigidBodies);
    }

    TestFalse(TEXT("The test backend is gone after unregistering"), UBot::Physics::FindBackendFactory(BackendName).IsValid());
    TestFalse(TEXT("And no longer listed"), UBot::Physics::GetBackendNames().Contains(BackendName));
    TestTrue(TEXT("The extension point itself stays"), Registry.IsExtensionPointDeclared(UBot::Physics::BackendExtensionPoint));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUBotPhysicsPackageDescriptorTest, "UBotPhysics.Package.Descriptor", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FUBotPhysicsPackageDescriptorTest::RunTest(const FString& Parameters)
{
    FUBotPackageRegistry Registry;
    Registry.Refresh();
    const FUBotPackageInfo* Package = Registry.FindPackage(TEXT("UBotPhysics"));
    if (!TestNotNull(TEXT("The package registry finds UBotPhysics"), Package))
    {
        return false;
    }

    TestEqual(TEXT("Version"), Package->Version, FString(TEXT("0.1.0")));
    TestEqual(TEXT("Layer"), Package->Layer, EUBotPackageLayer::Capability);
    TestEqual(TEXT("Provides"), FString::Join(Package->Provides, TEXT(",")), FString(TEXT("Physics.Backends,Physics.BodyBinding")));
    TestEqual(TEXT("Tags"), FString::Join(Package->Tags, TEXT(",")), FString(TEXT("physics")));
    TestTrue(TEXT("No engine plugins"), Package->ExternalRequires.IsEmpty());
    if (TestEqual(TEXT("One requirement"), Package->Requires.Num(), 1))
    {
        TestEqual(TEXT("Requires UBotCore"), Package->Requires[0].Name, FString(TEXT("UBotCore")));
        TestEqual(TEXT("At 0.1.3 or later in the 0.1 series"), Package->Requires[0].Version, FString(TEXT("^0.1.3")));
        TestFalse(TEXT("The requirement is not optional"), Package->Requires[0].bOptional);
    }
    TestTrue(TEXT("The descriptor has no problems"), Package->Problems.IsEmpty());
    return true;
}

#endif
