package cn.acg138.rover;

/** TextureView初始纹理覆盖整个视图，将其变换为等比例单眼裁剪区。 */
final class VideoGeometry {
    static float[] matrix(int width,int height,String view){
        if(width<=0||height<=0)throw new IllegalArgumentException("video area");
        float sourceWidth="stereo".equals(view)?1280:640;
        float sourceLeft="right".equals(view)?640:0;
        float scale=Math.min(width/sourceWidth,height/720f);
        return new float[]{scale*1280/width,0,(width-sourceWidth*scale)/2-sourceLeft*scale,
                           0,scale*720/height,(height-720*scale)/2,0,0,1};
    }
}
