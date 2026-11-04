package cn.acg138.rover;

/** 原生严格解析器使用固定五字段载荷，纯Java实现供主机测试。 */
final class NavigationPayload {
    private NavigationPayload() {}
    static String build(String action,String mapId,String version,double x,double y,
                        int tagId,double standoff,long sequence) {
        if(mapId==null||version==null||!mapId.matches("[0-9]{1,20}")||!version.matches("[0-9]{1,20}")||
           sequence<=0||sequence>9007199254740991L)throw new IllegalArgumentException("Invalid map context");
        String common="{\"map_id\":\""+mapId+"\",\"map_version\":\""+version+"\",\"seq\":"+sequence;
        if("tag".equals(action)) {
            if(tagId<0||tagId>586||!Double.isFinite(standoff)||standoff<0.2||standoff>3.0)
                throw new IllegalArgumentException("Invalid tag goal");
            return common+",\"tag_id\":"+tagId+",\"standoff_m\":"+Double.toString(standoff)+"}";
        }
        if(!("goal".equals(action)||"start".equals(action)||"stop".equals(action)||"reset".equals(action))||
           !Double.isFinite(x)||!Double.isFinite(y)||Math.abs(x)>100000||Math.abs(y)>100000)
            throw new IllegalArgumentException("Invalid map goal");
        return common+",\"x\":"+Double.toString(x)+",\"y\":"+Double.toString(y)+"}";
    }
}
