import org.jbox2d.dynamics.joints.Joint;
import processing.core.*;
import KinectPV2.*;
import shiffman.box2d.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;

import java.util.ArrayList;

public class Main extends PApplet {
    public enum Type {
        STATIC,
        POINT2POINT
    }

    KinectPV2 kinect;
    Box2DProcessing box2d;
    ArrayList<Boundary> boundaries;
    ArrayList<Box> boxes;
    ArrayList<PointBoundary> pointBoundaries;
    PImage lastFace;

    public static void main(String[] args) {
        PApplet.main(new String[]{"--present", "Main"});
    }

    public void settings() {
        size(displayWidth, displayHeight, P2D);
    }

    public void setup() {
        background(0);

        box2d = new Box2DProcessing(this);
        box2d.createWorld();
        box2d.setGravity(0,-10);

        boxes = new ArrayList<>();
        boundaries = new ArrayList<>();
        pointBoundaries = new ArrayList<>();
        for(int i = 0; i < 24; i++) pointBoundaries.add(new PointBoundary(0,0,0,50,50));

        boundaries.add(new Boundary(displayWidth / 2, displayHeight - 25, displayWidth, 50, 0));

        kinect = new KinectPV2(this);
        kinect.enableSkeletonDepthMap(true);
        kinect.enableDepthImg(true);
        kinect.enableBodyTrackImg(true);
        kinect.enableInfraredLongExposureImg(true);
        kinect.enableColorImg(true);
        kinect.enableSkeletonColorMap(true);
        kinect.enableFaceDetection(true);
        kinect.enableInfraredImg(true);

        kinect.init();

    }

    public void draw() {
        background(0);

        box2d.step();

        kinect.generateFaceData();

        image(kinect.getColorImage(),0,0,displayWidth,displayHeight);

        ArrayList<KSkeleton> skeletonArray = kinect.getSkeletonColorMap();

        boolean found = false;
        for (int i = 0; i < skeletonArray.size(); i++) {
            KSkeleton skeleton = skeletonArray.get(i);
            if (skeleton.isTracked() && !found) {
                found = true;
                KJoint[] joints = skeleton.getJoints();

                // Middle
                drawBone(joints, KinectPV2.JointType_Head, KinectPV2.JointType_Neck,0);
                drawBone(joints, KinectPV2.JointType_Neck, KinectPV2.JointType_SpineShoulder,1);
                drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_SpineMid,2);
                drawBone(joints, KinectPV2.JointType_SpineMid, KinectPV2.JointType_SpineBase,3);
                drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderRight,4);
                drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderLeft,5);
                drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipRight,6);
                drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipLeft,7);

                // Right Arm
                drawBone(joints, KinectPV2.JointType_ShoulderRight, KinectPV2.JointType_ElbowRight,8);
                drawBone(joints, KinectPV2.JointType_ElbowRight, KinectPV2.JointType_WristRight,9);
                drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_HandRight,10);

                // Left Arm
                drawBone(joints, KinectPV2.JointType_ShoulderLeft, KinectPV2.JointType_ElbowLeft,13);
                drawBone(joints, KinectPV2.JointType_ElbowLeft, KinectPV2.JointType_WristLeft,14);
                drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_HandLeft,15);

                // Right Leg
                drawBone(joints, KinectPV2.JointType_HipRight, KinectPV2.JointType_KneeRight,18);
                drawBone(joints, KinectPV2.JointType_KneeRight, KinectPV2.JointType_AnkleRight,19);
                drawBone(joints, KinectPV2.JointType_AnkleRight, KinectPV2.JointType_FootRight,20);

                // Left Leg
                drawBone(joints, KinectPV2.JointType_HipLeft, KinectPV2.JointType_KneeLeft,21);
                drawBone(joints, KinectPV2.JointType_KneeLeft, KinectPV2.JointType_AnkleLeft,22);
                drawBone(joints, KinectPV2.JointType_AnkleLeft, KinectPV2.JointType_FootLeft,23);

                for(PointBoundary pb : pointBoundaries){
                    pb.display();
                }

                // check if hands are closed, then spawn boxes
                KJoint leftHand = joints[KinectPV2.JointType_HandLeft];
                KJoint rightHand = joints[KinectPV2.JointType_HandRight];

                int leftHandState = leftHand.getState();
                int rightHandState = rightHand.getState();

                if(leftHandState == KinectPV2.HandState_Closed && rightHandState == KinectPV2.HandState_Closed){
                    buildBoxes();
                }

            }


            // save face if one exists
            saveFace();
        }

        // Display all the boundaries
        for (Boundary wall : boundaries) {
            wall.display();
        }

        // Display all the boxes
        for (Box b : boxes) {
            b.display();
        }

        // Boxes that leave the screen, we delete them
        // (note they have to be deleted from both the box2d world and our list
        for (int i = boxes.size() - 1; i >= 0; i--) {
            Box b = boxes.get(i);
            if (b.done()) {
                boxes.remove(i);
            }
        }


    }

    public void saveFace(){
        ArrayList<FaceData> faceData =  kinect.getFaceData();
        PImage currentFrame = kinect.getColorImage();

        float x = 0;
        float y = 0;
        float w = 0;
        float h = 0;

        boolean found = false;
        for (int i = 0; i < faceData.size(); i++) {
            FaceData faceD = faceData.get(i);
            if (faceD.isFaceTracked() && !found) {
                found = true;

                KRectangle faceRec = faceD.getBoundingRectColor();
                x = faceRec.getX();
                y = faceRec.getY();
                w = faceRec.getWidth();
                h = faceRec.getHeight();
            }
        }

        if(found) {
            lastFace = currentFrame.get(round(x), round(y), round(w), round(h));
            fill(255,0,0);
            ellipse(map(x,0,1920,0,displayWidth)+w/2,map(y,0,1080,0,displayHeight)+h/2,50,50);
        }
    }

    public void buildBoxes(){
        for(Box b : boxes){
            b.killBody();
        }
        boxes.clear();

        if(lastFace == null) return;

        float xPos = 0;
        float yPos = 0;
        float x_off = displayWidth/2;
        float y_off = 0;
        int blockSize = 6;
        PImage tempImage = null;

        while(xPos < (lastFace.width+blockSize)){
            yPos = 0;
            while (yPos < (lastFace.height+blockSize)){
                tempImage = lastFace.get(round(xPos),round(yPos), blockSize, blockSize).copy();
                Box p = new Box(xPos+x_off, yPos+y_off, blockSize, blockSize, tempImage);
                boxes.add(p);
                yPos+=blockSize;
            }
            xPos+=blockSize;
        }
    }

    public void keyPressed() {
        if (key == CODED) {
            if (keyCode == UP) {
                buildBoxes();
            }
        }
    }

    public float mapX(float kinectPosition) {
        if(!Float.isFinite(kinectPosition)) return -1;
        else return map(kinectPosition, 0, 1920, 0, displayWidth);
    }

    public float mapY(float kinectPosition) {
        if(!Float.isFinite(kinectPosition)) return -1;
        else return map(kinectPosition, 0, 1080, 0, displayHeight);
    }

    public void drawBone(KJoint[] joints,int type1,int type2, int index){
        KJoint j1 = joints[type1];
        KJoint j2 = joints[type2];

        float x1 = mapX(j1.getX());
        float y1 = mapY(j1.getY());
        float x2 = mapX(j2.getX());
        float y2 = mapY(j2.getY());

        if(x1 == -1 || y1 == -1 || x2 == -1 || y2 == -1) return;

        pointBoundaries.get(index).setPosition(x1,y1,x2,y2);

        fill(29,185,84);
        ellipse(x1,y1,50,50);
    }

    class Box {

        // We need to keep track of a Body and a width and height
        Body body;
        float w;
        float h;
        PImage img;

        // Constructor
        Box(float x, float y, float w_, float h_, PImage img_) {
            w = w_;
            h = h_;
            img = img_;
            makeBody(new Vec2(x, y), w, h);

        }

        // This function removes the particle from the box2d world
        void killBody() {
            box2d.destroyBody(body);
        }

        // Is the particle ready for deletion?
        boolean done() {
            // Let's find the screen position of the particle
            Vec2 pos = box2d.getBodyPixelCoord(body);
            // Is it off the bottom of the screen?
            if (pos.y > height + w * h) {
                killBody();
                return true;
            }
            return false;
        }

        // Drawing the box
        void display() {
            // We look at each body and get its screen position
            Vec2 pos = box2d.getBodyPixelCoord(body);
            // Get its angle of rotation
            float a = body.getAngle();

            rectMode(CENTER);
            pushMatrix();
            translate(pos.x, pos.y);
            rotate(-a);
            noFill();
            noStroke();
            beginShape();
            textureMode(IMAGE);
            if(img != null)texture(img);
            vertex(0,0);
            vertex(w,0);
            vertex(w,h);
            vertex(0,h);
            endShape();
            popMatrix();
        }

        // This function adds the rectangle to the box2d world
        void makeBody(Vec2 center, float w_, float h_) {

            // Define a polygon (this is what we use for a rectangle)
            PolygonShape sd = new PolygonShape();
            float box2dW = box2d.scalarPixelsToWorld(w_ / 2);
            float box2dH = box2d.scalarPixelsToWorld(h_ / 2);
            sd.setAsBox(box2dW, box2dH);

            // Define a fixture
            FixtureDef fd = new FixtureDef();
            fd.shape = sd;
            // Parameters that affect physics
            fd.density = 1;
            fd.friction = (float) 0.3;
            fd.restitution = (float) 0.5;

            // Define the body and make it from the shape
            BodyDef bd = new BodyDef();
            bd.type = BodyType.DYNAMIC;
            bd.position.set(box2d.coordPixelsToWorld(center));

            body = box2d.createBody(bd);
            body.createFixture(fd);
            //body.setMassFromShapes();
        }

    }

    class Boundary {

        // A boundary is a simple rectangle with x,y,width,and height
        float x;
        float y;
        float w;
        float h;
        // But we also have to make a body for box2d to know about it
        Body b;

        Boundary(float x_, float y_, float w_, float h_, float a) {
            x = x_;
            y = y_;
            w = w_;
            h = h_;

            // Define the polygon
            PolygonShape sd = new PolygonShape();
            // Figure out the box2d coordinates
            float box2dW = box2d.scalarPixelsToWorld(w / 2);
            float box2dH = box2d.scalarPixelsToWorld(h / 2);
            // We're just a box
            sd.setAsBox(box2dW, box2dH);


            // Create the body
            BodyDef bd = new BodyDef();
            bd.type = BodyType.STATIC;
            bd.angle = a;
            bd.position.set(box2d.coordPixelsToWorld(x, y));
            b = box2d.createBody(bd);

            // Attached the shape to the body using a Fixture
            b.createFixture(sd, 1);
        }

        void setPosition(float xPos,float yPos){
            x = xPos;
            y = yPos;
            b.setTransform(box2d.coordPixelsToWorld(x, y),b.getAngle());
        }

        // Draw the boundary, if it were at an angle we'd have to do something fancier
        void display() {
            fill(64,64,64);
            rectMode(CENTER);
            float a = b.getAngle();
            pushMatrix();
            translate(x, y);
            rotate(-a);
            rect(0, 0, w, h);
            popMatrix();
        }

    }

    class PointBoundary {

        // A boundary is a simple rectangle with x,y,width,and height
        float x1;
        float y1;
        float x2;
        float y2;
        float h;

        float w;
        float c_x;
        float c_y;
        float a;

        // But we also have to make a body for box2d to know about it
        Body b;
        PolygonShape sd;
        Fixture fixture;

        PointBoundary(float x1_, float y1_, float x2_, float y2_,float h_) {
            x1 = x1_;
            y1 = y1_;
            x2 = x2_;
            y2 = y2_;
            h = h_;

            w = dist(x1,y1,x2,y2);
            c_x = (x1+x2)/2;
            c_y = (y1+y2)/2;
            a = atan2(x2-x1,y2-y1)+PI/2;

            // Define the polygon
            sd = new PolygonShape();
            // Figure out the box2d coordinates
            float box2dW = box2d.scalarPixelsToWorld(w / 2);
            float box2dH = box2d.scalarPixelsToWorld(h / 2);
            // We're just a box
            sd.setAsBox(box2dW, box2dH);


            // Create the body
            BodyDef bd = new BodyDef();
            bd.type = BodyType.STATIC;
            bd.angle = a;
            bd.position.set(box2d.coordPixelsToWorld(c_x,c_y));
            b = box2d.createBody(bd);

            // Attached the shape to the body using a Fixture
            fixture = b.createFixture(sd, 1);
        }

        void setPosition(float x1_, float y1_, float x2_, float y2_){
            x1 = x1_;
            y1 = y1_;
            x2 = x2_;
            y2 = y2_;
            w = dist(x1,y1,x2,y2);
            c_x = (x1+x2)/2;
            c_y = (y1+y2)/2;
            a = atan2(x2-x1,y2-y1)+PI/2;
            b.setTransform(box2d.coordPixelsToWorld(c_x,c_y),a);

            // NEEDS SUPER COMPUTER TO DO BELOW APPARENTLY
            float box2dW = box2d.scalarPixelsToWorld(w / 2);
            float box2dH = box2d.scalarPixelsToWorld(h / 2);

            PolygonShape ps = (PolygonShape) fixture.getShape();
            ps.setAsBox(box2dW,box2dH);
        }

        // Draw the boundary, if it were at an angle we'd have to do something fancier
        void display() {
            fill(40,40,40);
            rectMode(CENTER);
            pushMatrix();
            translate(c_x,c_y);
            rotate(-a);
            rect(0, 0, w, h);
            popMatrix();
        }

    }

}