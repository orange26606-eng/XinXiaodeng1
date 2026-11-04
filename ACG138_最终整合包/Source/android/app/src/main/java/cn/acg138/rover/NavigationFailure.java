package cn.acg138.rover;

/* 区分HTTP传输错误与当前任务状态。 */
public final class NavigationFailure extends Exception {
    private static final long serialVersionUID=1L;
    public final String operation, code;
    public final int httpStatus;
    public NavigationFailure(String operation,String code,int status,String message) {
        super(message); this.operation=operation;this.code=code;this.httpStatus=status;
    }
    public static NavigationFailure http(String operation,int status,String serverCode,String message) {
        String fallback=status==401?"SESSION_EXPIRED":status==409?"REQUEST_CONFLICT":status==400?"REQUEST_INVALID":status==503?("map".equals(operation)?"MAP_UNAVAILABLE":"NAV_SERVICE_UNAVAILABLE"):"HTTP_ERROR";
        String code=serverCode!=null&&serverCode.matches("[A-Z0-9_]{1,64}")?serverCode:fallback;
        String text=message==null||message.length()==0?"请求未完成，请查看状态后重试":message;
        return new NavigationFailure(operation,code,status,text.substring(0,Math.min(200,text.length())));
    }
    public static NavigationFailure from(String operation,Exception error) {
        if(error instanceof NavigationFailure)return (NavigationFailure)error;
        boolean timeout=error instanceof java.net.SocketTimeoutException;
        String code=timeout?"REQUEST_TIMEOUT":error instanceof java.io.IOException?"NETWORK_ERROR":"RESPONSE_INVALID";
        return new NavigationFailure(operation,code,0,timeout?"请求超时；不代表底盘反馈故障":code.equals("NETWORK_ERROR")?"网络请求失败；请检查连接":"返回数据不兼容，请使用配套版本");
    }
}
