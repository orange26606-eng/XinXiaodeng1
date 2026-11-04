package cn.acg138.rover;

public final class AutoSwitchTest {
    private static int checks;
    private static final String TOKEN = "0123456789abcdef0123456789abcdef";

    private static void check(boolean condition) {
        if (!condition) throw new AssertionError("check " + (checks + 1));
        checks++;
    }

    private static void rejected(String token, long issued, long expires) {
        try {
            new AutoSwitch.Prepared(token, issued, expires);
            throw new AssertionError("invalid receipt accepted");
        } catch (IllegalArgumentException expected) {
            checks++;
        }
    }

    private static final class Guard implements AutoSwitch.Guard {
        long time = 100, sequence;
        boolean active = true;
        public boolean current() { return active; }
        public long now() { return time; }
        public long nextSequence() { return ++sequence; }
    }

    private static final class Transport implements AutoSwitch.Transport {
        final Guard guard;
        int prepared, started;
        boolean cancelAfterPrepare, expireAfterPrepare, badAck, rejectedAck;
        Transport(Guard guard) { this.guard = guard; }
        public AutoSwitch.Prepared prepare(long sequence, int timeout) {
            check(timeout > 0 && timeout <= AutoSwitch.HTTP_MS);
            check(sequence == 1);
            prepared++;
            if (cancelAfterPrepare) guard.active = false;
            if (expireAfterPrepare) guard.time = 2600;
            return new AutoSwitch.Prepared(TOKEN, 900000000, 908000000);
        }
        public AutoSwitch.Accepted start(long sequence, String token, int timeout) {
            check(sequence == 2 && token.equals(TOKEN));
            check(timeout > 0 && timeout <= AutoSwitch.HTTP_MS);
            started++;
            return new AutoSwitch.Accepted(!rejectedAck, badAck ? 900000001 : 900000000);
        }
    }

    private static void failed(Transport transport, Guard guard) throws Exception {
        try {
            AutoSwitch.execute(transport, guard, 2600);
            throw new AssertionError("failed transaction accepted");
        } catch (java.io.InterruptedIOException expected) {
            checks++;
        }
    }

    public static void main(String[] args) throws Exception {
        check(new AutoSwitch.Prepared(TOKEN, 1, 8000001).expiresUs == 8000001);
        check(new AutoSwitch.Prepared(TOKEN, 1, 2500001).expiresUs == 2500001);
        rejected(TOKEN, 1, 8000002);
        rejected(TOKEN, 0, 8000000);
        rejected(TOKEN, 1, 1);
        rejected(TOKEN, 2, 1);
        rejected(null, 1, 2);
        rejected("bad", 1, 2);
        rejected(TOKEN.toUpperCase(), 1, 2);
        Guard guard = new Guard();
        Transport transport = new Transport(guard);
        check(AutoSwitch.execute(transport, guard, 2600) == 900000000);
        check(transport.prepared == 1 && transport.started == 1);
        guard = new Guard(); transport = new Transport(guard);
        guard.active = false;
        failed(transport, guard);
        check(transport.prepared == 0 && transport.started == 0);
        guard = new Guard(); transport = new Transport(guard);
        transport.cancelAfterPrepare = true;
        failed(transport, guard);
        check(transport.prepared == 1 && transport.started == 0);
        guard = new Guard(); transport = new Transport(guard);
        transport.expireAfterPrepare = true;
        failed(transport, guard);
        check(transport.started == 0);
        for (int i = 0; i < 2; i++) {
            guard = new Guard(); transport = new Transport(guard);
            transport.badAck = i == 0; transport.rejectedAck = i == 1;
            try {
                AutoSwitch.execute(transport, guard, 2600);
                throw new AssertionError("bad acknowledgement accepted");
            } catch (Exception expected) {
                check(expected.getMessage().contains("不匹配"));
            }
        }
        System.out.println("AutoSwitchTest PASS checks=" + checks);
    }
}
