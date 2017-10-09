package com.c2_systems.benchmark;

import java.util.Arrays;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.BenchmarkMode;
import org.openjdk.jmh.annotations.Fork;
import org.openjdk.jmh.annotations.Measurement;
import org.openjdk.jmh.annotations.Mode;
import org.openjdk.jmh.annotations.OutputTimeUnit;
import org.openjdk.jmh.annotations.Param;
import org.openjdk.jmh.annotations.Scope;
import org.openjdk.jmh.annotations.Setup;
import org.openjdk.jmh.annotations.State;
import org.openjdk.jmh.annotations.Warmup;
import org.openjdk.jmh.infra.Blackhole;

import io.reactivex.Flowable;
import io.reactivex.disposables.Disposable;
import io.reactivex.functions.Function;
import io.reactivex.schedulers.Schedulers;

@BenchmarkMode(Mode.Throughput)
@Warmup(iterations = 5)
@Measurement(iterations = 5, time = 1, timeUnit = TimeUnit.SECONDS)
//@Fork(value = 1,jvmArgsAppend = { "-XX:MaxInlineLevel=20" })
@OutputTimeUnit(TimeUnit.SECONDS)
@State(Scope.Thread)
public class MyBenchmark implements Function<Integer, Integer> {

    @Param({"2000", "4000", "6000"}) //10000
    public int count;

    @Param({"50", "100", "250", "500", "1000"})
    public int compute;

    @Param({"3"})
    public int parallelism;

    Integer[] ints;

    Random randomNum = new Random();

    Flowable<Integer> parallel;
    Flowable<Integer> notsoparallel;

    @Override
    public Integer apply(Integer t) throws Exception {
        Blackhole.consumeCPU(compute);
        return t;
    }

    @Setup
    public void setup() {

        final int cpu = parallelism;
        ints = new Integer[count];

        for(int i = 0; i<count; i++) {
        	ints[i] = randomNum.nextInt(2);
        }

        Arrays.fill(ints, 777);

        Flowable<Integer> source = Flowable.fromArray(ints);

        /*another = source.groupBy((i) -> 0 == (int)i % 2 ? "EVEN" : "ODD")
                .subscribe((group) -> {
                    System.out.println("Key " + ((GroupedFlowable<String,Integer>)group).getKey());
                    ((GroupedFlowable<String,Integer>)group).subscribe((x) -> System.out.println(((GroupedFlowable<String,Integer>)group).getKey() + ": " + x));
                });*/

        parallel = source.parallel(cpu).runOn(Schedulers.computation()).map(this).sequential();

        notsoparallel = source.map(this);


    }


    void subscribe(Flowable<Integer> f, Blackhole bh) {
        PerfAsyncConsumer consumer = new PerfAsyncConsumer(bh);
        f.subscribe(consumer);
        consumer.await(count);
    }

    public void other(Blackhole bh) {
    	for(int i = 0; i<count; i++) {
			Blackhole.consumeCPU(compute);
		}
    }

    @Benchmark
    public void parallel(Blackhole bh) {
        subscribe(parallel, bh);
    }

    @Benchmark
    public void notsoparallel(Blackhole bh) {
        subscribe(notsoparallel, bh);
    }


    @Benchmark
    public void oldschool(Blackhole bh) {

    	ExecutorService executor = Executors.newFixedThreadPool(parallelism);

    	try {
    		for(int i =0; i<ints.length; i++) {
        		executor.submit(() -> {
        			Blackhole.consumeCPU(compute);
        		});
        	}
    	} catch (Exception e) {
            throw new IllegalStateException(e);
        } finally {
    		executor.shutdown();
    	}

    }

    @Benchmark
    public void loopy(Blackhole bh) {

		for(int i =0; i<ints.length; i++) {
    		Blackhole.consumeCPU(compute);
    	}

    }
}