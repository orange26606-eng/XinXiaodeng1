package cn.acg138.rover;

/** 只读实物选择载荷，不包含导航或电机操作。 */
final class ObjectPayload {
    private ObjectPayload() {}
    static String build(String operation,long seq,String epoch,String snapshotId,int trackId) {
        if(seq<=0||seq>9007199254740991L)throw new IllegalArgumentException("Invalid object sequence");
        if("clear".equals(operation))return "{\"seq\":"+seq+"}";
        if(!"select".equals(operation)||epoch==null||!epoch.matches("[1-9][0-9]{0,19}")||
           snapshotId==null||!snapshotId.matches("[1-9][0-9]{0,19}-[1-9][0-9]{0,19}")||
           !snapshotId.startsWith(epoch+"-")||trackId<1)
            throw new IllegalArgumentException("Invalid object identity");
        try{Long.parseUnsignedLong(epoch);Long.parseUnsignedLong(snapshotId.substring(epoch.length()+1));}
        catch(NumberFormatException e){throw new IllegalArgumentException("Object identity exceeds uint64");}
        return "{\"seq\":"+seq+",\"epoch\":\""+epoch+"\",\"snapshot_id\":\""+snapshotId+"\",\"track_id\":"+trackId+"}";
    }
}
