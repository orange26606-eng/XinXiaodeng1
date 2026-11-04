package cn.acg138.rover;
import java.util.concurrent.*;

public final class RevocationTest {
    public static void main(String[] args) throws Exception {
        SemanticTaskService.revocation = null;
        SemanticTaskService.awaitRevocationForAuto();
        CompletableFuture<Void> delayed = new CompletableFuture<>();
        SemanticTaskService.revocation = delayed;
        Thread completion = new Thread(() -> {
            try { Thread.sleep(60); } catch (InterruptedException error) { throw new RuntimeException(error); }
            delayed.complete(null);
        });
        completion.start();
        SemanticTaskService.awaitRevocationForAuto();
        if (!delayed.isDone()) throw new AssertionError("cancel barrier incomplete");
        completion.join();
        CompletableFuture<Void> failed = new CompletableFuture<>();
        failed.completeExceptionally(new IllegalStateException("cancel failed"));
        SemanticTaskService.revocation = failed;
        try {
            SemanticTaskService.awaitRevocationForAuto();
            throw new AssertionError("failed cancellation ignored");
        } catch (ExecutionException expected) { }
        SemanticTaskService.revocation = new CompletableFuture<>();
        try {
            SemanticTaskService.awaitRevocationForAuto();
            throw new AssertionError("pending cancellation ignored");
        } catch (java.io.IOException expected) {
            if (!(expected.getCause() instanceof TimeoutException)) throw expected;
        }
        System.out.println("RevocationTest PASS cases=4");
    }
}
