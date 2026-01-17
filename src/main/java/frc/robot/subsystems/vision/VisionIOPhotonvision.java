package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhotonvision implements VisionIO {
    protected final PhotonCamera camera;
	protected final Transform3d robotToCamera;

	public VisionIOPhotonvision(String name, Transform3d robotToCamera) {
		camera = new PhotonCamera(name);
		this.robotToCamera = robotToCamera;
	}
}
