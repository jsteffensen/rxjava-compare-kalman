package com.c2_systems.benchmark;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.random.JDKRandomGenerator;
import org.apache.commons.math3.random.RandomGenerator;
import org.openjdk.jmh.annotations.Benchmark;
import org.openjdk.jmh.annotations.BenchmarkMode;
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
import io.reactivex.functions.Function;

@BenchmarkMode(Mode.Throughput)
@Warmup(iterations = 5)
@Measurement(iterations = 5, time = 1, timeUnit = TimeUnit.SECONDS)
//@Fork(value = 1,jvmArgsAppend = { "-XX:MaxInlineLevel=20" })
@OutputTimeUnit(TimeUnit.SECONDS)
@State(Scope.Thread)
public class MyBenchmark implements Function<Integer, Integer> {

    @Param({"1000"})
    public int count;

    @Param({"50", "100", "250", "500", "1000"})
    public int compute;

    Integer[] ints;

    Flowable<Integer> notsoparallel;

    @Override
    public Integer apply(Integer t) throws Exception {

    	//--------------------------------------------------
        // Create filter
        //--------------------------------------------------

        double dt = 0.1d;
        double measurementNoise = 10d;
        double accelNoise = 0.2d;

        RealMatrix A = new Array2DRowRealMatrix(new double[][] { { 1, dt }, { 0, 1 } });

        RealMatrix B = new Array2DRowRealMatrix(new double[][] { { Math.pow(dt, 2d) / 2d }, { dt } });

        RealMatrix H = new Array2DRowRealMatrix(new double[][] { { 1d, 0d } });

        RealVector x = new ArrayRealVector(new double[] { 0, 0 });

        RealMatrix tmp = new Array2DRowRealMatrix(new double[][] {
            { Math.pow(dt, 4d) / 4d, Math.pow(dt, 3d) / 2d },
            { Math.pow(dt, 3d) / 2d, Math.pow(dt, 2d) } });

        RealMatrix Q = tmp.scalarMultiply(Math.pow(accelNoise, 2));

        RealMatrix P0 = new Array2DRowRealMatrix(new double[][] { { 1, 1 }, { 1, 1 } });

        RealMatrix R = new Array2DRowRealMatrix(new double[] { Math.pow(measurementNoise, 2) });

        RealVector u = new ArrayRealVector(new double[] { 0.1d });

        ProcessModel pm = new DefaultProcessModel(A, B, Q, x, P0);
        MeasurementModel mm = new DefaultMeasurementModel(H, R);
        KalmanFilter filter = new KalmanFilter(pm, mm);

        RandomGenerator rand = new JDKRandomGenerator();

        RealVector tmpPNoise = new ArrayRealVector(new double[] { Math.pow(dt, 2d) / 2d, dt });
        RealVector mNoise = new ArrayRealVector(1);

        //--------------------------------------------------
        // Use filter
        //--------------------------------------------------

        filter.predict(u);
        RealVector pNoise = tmpPNoise.mapMultiply(accelNoise * rand.nextGaussian());
        x = A.operate(x).add(B.operate(u)).add(pNoise);
        mNoise.setEntry(0, measurementNoise * rand.nextGaussian());
        RealVector z = H.operate(x).add(mNoise);

        filter.correct(z);

        Double position = filter.getStateEstimation()[t];
        Double velocity = filter.getStateEstimation()[t+1];

        // return a result in order to not shortcut benchmark
        return position.intValue() + velocity.intValue();

    }

    /*@Override
    public Integer apply(Integer t) throws Exception {
        Blackhole.consumeCPU(compute);
        return t;
    }*/

    @Setup
    public void setup() {

        ints = new Integer[count];
        Arrays.fill(ints, 777);

        Flowable<Integer> source = Flowable.fromArray(ints);

        notsoparallel = source.map(this);

    }


    void subscribe(Flowable<Integer> f, Blackhole bh) {
        PerfAsyncConsumer consumer = new PerfAsyncConsumer(bh);
        f.subscribe(consumer);
        consumer.await(count);
    }

    @Benchmark
    public void notsoparallel(Blackhole bh) {
        subscribe(notsoparallel, bh);
    }



}