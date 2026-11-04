// 由policy/r6-motion.json生成，仅用于显示及会话标识。
package cn.acg138.rover;
final class MotionIdentity {
    static final long CONFIG_ID=0x31110301L;
    static final int STATUS_REVISION=5;
    static boolean sessionMatches(long config,int status){return config==CONFIG_ID&&status==STATUS_REVISION;}
    static String mismatch(long config,int status){return String.format(java.util.Locale.ROOT,"版本指纹不匹配：本App要求CFG%08X／状态%d，板端回报CFG%08X／状态%d，请成套更新FS和RK程序",CONFIG_ID,STATUS_REVISION,config,status);}
}
